#include <queue>
std::queue<String> highPriorityQueue;  // Mensajes críticos (END, ACK)
std::queue<String> lowPriorityQueue;   // Mensajes normales (TEL)

#include "LoRaWan_APP.h"
#include "HT_SSD1306Wire.h"
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>
// CONFIGURACIÓN EEPROM
#define EEPROM_SIZE 64
#define EEPROM_ID_ADDRESS 0
// ================= CONFIGURACIÓN ====================
#define USE_OLED true
#define BUFFER_SIZE 128
#define TX_OUTPUT_POWER 14

// Configuración LoRa
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define LORA_SYMBOL_TIMEOUT 0

// Configuración TDMA
#define TDMA_SLOT_DURATION 20000  // 3 segundos por slot
#define TOTAL_SLOTS 3            // 0=Base, 1-2=Remotos
#define BASE_FIJA_ID 0
#define ACK_TIMEOUT 12000         // 5 segundos para ACK

// Frecuencias
#define FREQ_BASE 869000000      // Base fija
#define FREQ_REMOTOS 915000000   // Entre remotos

//BATERIA==============================
#define VBAT_PIN 1          // GPIO1 (ADC1_CH0)
#define ADC_CTRL_PIN 37     // Pin interno de control (NO conectar externamente)
#define BATTERY_SAMPLES 10

// Pines
#define LED_PIN 48
#define RELE_PIN 26
#define LED_PINB 20
#define RELE_PINB 19
#define MOTOR_A1 LED_PIN    // Motor Izquierdo Robot 133333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333
#define MOTOR_A2 RELE_PIN   // Motor Derecho Robot 1
#define MOTOR_B1 LED_PINB    // Motor Izquierdo Robo+t 1
#define MOTOR_B2 RELE_PINB
// ================= CONFIGURACIÓN BME280 ==============
TwoWire Wire2 = TwoWire(1);  
Adafruit_BME280 bme;
#define BME_SDA 41
#define BME_SCL 42
#define BME_ADDRESS 0x76

// ================= CONFIG TOKEN RING ================
#define TOKEN_TIMEOUT 10000       // 10 seg timeout
#define SYNC_INTERVAL 30000      // 30 seg
#define MAX_MISSED_SYNCS 5

#include <WiFi.h>
#include <WebServer.h>

// Configuración Web Server (solo base)
#define MAX_REMOTES 2
struct RemoteNode {
    int id = 0;
    float temperature;
    float humidity;
    float pressure;
    float altitude;
    float voltage;
    float batteryPercent;
    int rssi;
    String lastStatus;
    unsigned long lastUpdate;
};
RemoteNode remotes[MAX_REMOTES];

const char* apSSID = "BaseControl";
const char* apPassword = "control123";
IPAddress localIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);
//

// presion de referencia
float referencePressure = 1013.25; // Presión al nivel del mar estándar (hPa)
float barometerAltitude = 1650.3;   // Altitud real del sensor (metros)

// ================= VARIABLES GLOBALES ================
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
bool lora_idle = true;
int deviceId = BASE_FIJA_ID;
String currentRoutine = "";
String systemStatus = "Operativo"; 
bool routineInProgress = false;
unsigned long routineStartTime = 0;
bool waitingForAck = false;
unsigned long ackWaitStart = 0;
bool waitingForSlot = false;
unsigned long slotStartTime = 0;


bool hasToken = false;
int nextNode = (deviceId + 1) % TOTAL_SLOTS;
unsigned long lastTokenActivity = 0;
unsigned long lastSync = 0;
int syncMissed = 0;
bool inSafing = false;


bool pendingConfirmation = false;
unsigned long lastEndSend = 0;
#define MAX_SEND_ATTEMPTS 3
int sendAttempts = 0;



// OLED
#ifdef WIRELESS_STICK_V3
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_64_32, RST_OLED);
#else
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
#endif

static RadioEvents_t RadioEvents;

void sendWithTDMA(const char* command, uint32_t freq, bool isHighPriority = false);

//CONFIGURACION DE EEPROM

void saveDeviceId() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(EEPROM_ID_ADDRESS, deviceId);
  EEPROM.commit();
  EEPROM.end();
}

void loadDeviceId() {
  EEPROM.begin(EEPROM_SIZE);
  
  // Verificar inicialización
  byte firstByte = EEPROM.read(EEPROM_ID_ADDRESS);
  if(firstByte == 0xFF) { // EEPROM virgen
    deviceId = BASE_FIJA_ID;
    saveDeviceId();
    EEPROM.end();
    return;
  }
  
  EEPROM.get(EEPROM_ID_ADDRESS, deviceId);
  
  // Validación
  if(deviceId < 0 || deviceId >= TOTAL_SLOTS) {
    deviceId = BASE_FIJA_ID;
    saveDeviceId();
  }
  
  EEPROM.end();
}




// ================= PROTOTIPOS ====================
String getValue(String data, int index);
String readSensorData();
void updateDisplay(String line1, String line2, String line3);
bool checkTxSlot(); 
// ================= SETUP =========================
void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}

void VBAT_Init() {
pinMode(VBAT_PIN, INPUT);
pinMode(ADC_CTRL_PIN, OUTPUT);
}


float readBattVoltage() {
digitalWrite(ADC_CTRL_PIN, LOW);

uint32_t raw = 0;
for (int i = 0; i < BATTERY_SAMPLES; i++) {
raw += analogRead(VBAT_PIN);
}
raw = raw / BATTERY_SAMPLES;
digitalWrite(ADC_CTRL_PIN, HIGH);
return 1.51714320988*3.90 * (3.3 / 1024.0) * raw;
}


void setup() {
  VextON();
  VBAT_Init();
  Serial.begin(115200);
  lastSync = millis();
  // Cargar ID desde EEPROM
  loadDeviceId();

  // Inicializar BME280 solo en dispositivos remotos
  if(deviceId != BASE_FIJA_ID) {
    Wire2.begin(BME_SDA, BME_SCL);
    while(!bme.begin(BME_ADDRESS, &Wire2)) {
      Serial.println("Falló BME280. Verificar conexiones para modo remoto.");
      delay(1000);
    }
    
    // Configuración del sampling DENTRO del bloque condicional
    bme.setSampling(
      Adafruit_BME280::MODE_NORMAL,
      Adafruit_BME280::SAMPLING_X2,
      Adafruit_BME280::SAMPLING_X16,
      Adafruit_BME280::SAMPLING_X1,
      Adafruit_BME280::FILTER_X16,
      Adafruit_BME280::STANDBY_MS_1000
    );
    // Iniciar token si es base fija o remota
    if(deviceId == BASE_FIJA_ID) {
        passToken();
    }

    Serial.println("\nSistema TDMA + BME280 Iniciado");
    printSystemInfo();

}

  // Resto de inicializaciones
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELE_PIN, OUTPUT);
    pinMode(LED_PINB, OUTPUT);
  pinMode(RELE_PINB, OUTPUT);
  
  if(USE_OLED) {
    display.init();
    display.clear();
    display.display();
  }

  // Inicializar LoRa
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
Radio.Init(&RadioEvents);
if(deviceId == BASE_FIJA_ID) {
    setFrequency(FREQ_BASE);
} else {
    setFrequency(FREQ_REMOTOS);
}

  Serial.println("\nSistema TDMA + BME280 Iniciado");
  printSystemInfo();

 if(deviceId == BASE_FIJA_ID) {
   setupWebServer();
  }
}

// ================= CHECKSUM ========================
uint8_t computeChecksum(const char* data, size_t len) {
    uint8_t checksum = 0;
    for(size_t i=0; i<len; i++) checksum ^= data[i];
    return checksum;
}

// ================= LOOP PRINCIPAL =================
void loop() {
  static unsigned long lastTelemetry = 0;
  
  // 1. Procesar comandos seriales (para todos los dispositivos)
  if(Serial.available()) {
    processSerialCommand(Serial.readStringUntil('\n'));
  }
    if(deviceId == BASE_FIJA_ID) {
    server.handleClient();
    // Enviar sync cada 30 seg
    if(millis() - lastSync > SYNC_INTERVAL) {
      sendCommand("SYNC", FREQ_BASE);
      lastSync = millis();
    }
  }

  // 2. Modo de operación base/remoto
  if(deviceId == BASE_FIJA_ID) {
    processBaseFijaMode();
  } else {
    processBaseRemotaMode();
    
    // 3. Envío periódico de telemetría solo para remotos
   if(millis() - lastTelemetry > 9000) {
        String telemetry = "TEL:" + String(deviceId) + ":" + readSensorData();
        sendWithTDMA(telemetry.c_str(), FREQ_BASE); // Prioridad normal por defecto
        lastTelemetry = millis();
    }
  }

  // 4. Ejecución de rutinas (común para ambos, base fija o remota)
  if(routineInProgress) {
    executeCurrentRoutine();
  }

  // 5. Gestión TDMA y ACK
  checkTDMA();
  checkAckTimeout();

  // 6. Procesar eventos LoRa
  Radio.IrqProcess();

  // 7. Actualizar pantalla con información relevante
  if(deviceId == BASE_FIJA_ID) {
    updateDisplay(currentRoutine, 
                 routineInProgress ? "En progreso" : "Esperando",
                 waitingForAck ? "ACK Pendiente" : "");
  } else {
    updateDisplay("Estado:", "Operativo", "");
  }

    // 8. Verificar sincronización y safing
    if(!inSafing && deviceId != BASE_FIJA_ID) {
        if(millis() - lastSync > SYNC_INTERVAL) {
            if(++syncMissed >= MAX_MISSED_SYNCS) {
                activateSafing();
            }
        }
    }

    if(pendingConfirmation && (millis() - lastEndSend > 2000)) {
        if(sendAttempts < MAX_SEND_ATTEMPTS && checkTxSlot()) {
            sendCommand(highPriorityQueue.front().c_str(), FREQ_BASE);
            sendAttempts++;
            lastEndSend = millis();
        } else {
            pendingConfirmation = false;
            highPriorityQueue.pop();
            Serial.println("Error: Fallo envio confirmacion");
        }
    }

if(deviceId == BASE_FIJA_ID && millis() - lastSync > SYNC_INTERVAL) {
        sendCommand("SYNC", FREQ_BASE);
        lastSync = millis();
        Serial.println("SYNC enviado");

    }



}


// ================= FUNCIONES BME280 ===============

// ================= CÁLCULO PUNTO DE ROCÍO ==============
float calculateDewPoint(float temp, float hum) {
  // Fórmula de Magnus-Tetens
  float a = 17.62;
  float b = 243.12;
  float alpha = log(hum / 100) + (a * temp) / (b + temp);
  return (b * alpha) / (a - alpha);
}


String readSensorData() {
  // Verificar si es dispositivo remoto Y el sensor está inicializado
  if(deviceId == BASE_FIJA_ID || !bme.sensorID()) { // Usar sensorID() como verificación
    return "NA,NA,NA,NA,NA,NA";
  }

  // Forzar modo de medición
  bme.takeForcedMeasurement();
  
  // Leer valores con verificación
  float temp = bme.readTemperature();
  float pres = bme.readPressure() / 100.0F;
  float hum = bme.readHumidity();
  float voltage = readBattVoltage();
  // Si hay valores inválidos
  if(isnan(temp) || isnan(pres) || isnan(hum)) {
    return "ERR,ERR,ERR,ERR,ERR,ERR";
  }

  // Cálculos
  float altitude = bme.readAltitude(referencePressure);
  float dewPoint = calculateDewPoint(temp, hum);
  float absHum = (6.112 * exp((17.67 * temp)/(temp + 243.5)) * hum * 2.1674) / (273.15 + temp);
  float pervolt = (voltage/4.17) * 100;
  // Formatear datos
 //char buffer[256];
//  snprintf(buffer, sizeof(buffer), 
 //   "\n"
//    "========= TELEMETRÍA [ID: %d] =========\n"
//    "-> T:%.1f°C H:%.1f%%\ P:%.1fhPa\n"
 //   "-> H:%.2fm V:%.1f %V:%.1f%%V "
  //  "========================================",
   // deviceId,
 //   temp, hum, pres,
  //  altitude,voltage,pervolt); 
 // return String(buffer);

   // Formatear para la base
  char buffer[128];
  snprintf(buffer, sizeof(buffer), 
    "T:%.1f,H:%.1f,P:%.2f,A:%.2f,V:%.2f,B:%.0f",
    temp, hum, pres, altitude, voltage, pervolt);
    
  return String(buffer);
}


String readSensorDataAux() {
  // Verificar si es dispositivo remoto Y el sensor está inicializado
  if(deviceId == BASE_FIJA_ID || !bme.sensorID()) { // Usar sensorID() como verificación
    return "NA,NA,NA,NA,NA,NA";
  }

  // Forzar modo de medición
  bme.takeForcedMeasurement();
  
  // Leer valores con verificación
  float temp = bme.readTemperature();
  float pres = bme.readPressure() / 100.0F;
  float hum = bme.readHumidity();
  float voltage = readBattVoltage();
  // Si hay valores inválidos
  if(isnan(temp) || isnan(pres) || isnan(hum)) {
    return "ERR,ERR,ERR,ERR,ERR,ERR";
  }

  // Cálculos
  float altitude = bme.readAltitude(referencePressure);
  float dewPoint = calculateDewPoint(temp, hum);
  float absHum = (6.112 * exp((17.67 * temp)/(temp + 243.5)) * hum * 2.1674) / (273.15 + temp);

  // Formatear datos
 char buffer[128];
  snprintf(buffer, sizeof(buffer), 
          "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",  // 7 valores separados por comas
          temp, hum, pres, altitude, dewPoint, absHum, voltage);
  
  return String(buffer);
}


// ================= FUNCIONES AUXILIARES ===========
String getValue(String data, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length();

  for(int i = 0; i <= maxIndex && found <= index; i++){
    if(data.charAt(i) == ',' || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void updateDisplay(String line1, String line2, String line3) {
  if(!USE_OLED) return;
  
  static unsigned long lastUpdate = 0;
  if(millis() - lastUpdate < 2000 && deviceId != BASE_FIJA_ID) return; // Solo aplicar delay en remotos
  
  display.clear();
  display.setFont(ArialMT_Plain_10);
  
  // Header común
  String header = (deviceId == BASE_FIJA_ID) ? "BASE FIJA" : "REMOTO ID:" + String(deviceId);
  display.drawString(0, 0, header);

  if(deviceId == BASE_FIJA_ID) {
    // ========= VISTA PARA BASE FIJA ==========
    display.drawString(0, 12, "Ultimo comando:");
    display.drawString(0, 24, line1);          // Ej: "Rutina completada"
    display.drawString(0, 36, line2);          // Ej: "RUTINA_1"
    display.drawString(0, 48, "Slot: " + String((millis()/TDMA_SLOT_DURATION)%TOTAL_SLOTS));
  } 
else {
    // ========= VISTA PARA REMOTOS ============
    String sensorData = readSensorDataAux();
    String voltageStr = getValue(sensorData, 6);
    
    // Ajustes clave:
    display.setFont(ArialMT_Plain_10); // Asegurar fuente pequeña
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    // Línea 1: Temperatura y Humedad
    display.drawString(0, 12, "T:" + getValue(sensorData, 0).substring(0,4) + "C");
    display.drawString(64, 12, "H:" + getValue(sensorData, 1).substring(0,4) + "%");

    // Línea 2: Presión y Altitud
    display.drawString(0, 24, "P:" + getValue(sensorData, 2).substring(0,6) + "hPa");
    display.drawString(64, 24, "A:" + getValue(sensorData, 3).substring(0,5) + "m");

    // Línea 3: Punto de rocío y Hum. Absoluta
    display.drawString(0, 36, "DP:" + getValue(sensorData, 4).substring(0,4) + "C");
    display.drawString(64, 36, "AH:" + getValue(sensorData, 5).substring(0,4) + "g");

    // Footer
    display.drawString(0, 48, "Slot:" + String((millis()/TDMA_SLOT_DURATION)%TOTAL_SLOTS));
    display.drawString(64, 48, "V:" + voltageStr.substring(0,4) + "V");
}

  display.display();
  lastUpdate = millis();
}

// ================= FUNCIONES COMUNICACIÓN ============
void sendCommand(const char* command, uint32_t freq) {
    char packet[BUFFER_SIZE];
    uint8_t checksum = computeChecksum(command, strlen(command));
    snprintf(packet, BUFFER_SIZE, "%s:%02X", command, checksum);
    
    setFrequency(freq);
    Serial.printf("Enviando [%lu MHz]: %s\n", freq/1000000, packet);
    Radio.Send((uint8_t *)packet, strlen(packet));
    lora_idle = false;
}


// ================= FUNCIÓN checkTxSlot ==============
bool checkTxSlot() {
    if(deviceId == BASE_FIJA_ID) return true;
    
    unsigned long currentTime = millis() - lastSync;
    if(currentTime > SYNC_INTERVAL * 2) return false; // Evitar slots desfasados
    
    unsigned long slotIndex = (currentTime / TDMA_SLOT_DURATION) % TOTAL_SLOTS;
    unsigned long slotTime = currentTime % TDMA_SLOT_DURATION;

    // Ventana de transmisión segura: primeros 2.5 segundos del slot
    return (slotIndex == (unsigned long)deviceId) && (slotTime < 2500);
}

// 3. Nueva función para enviar confirmaciones críticas
void sendCriticalMessage(String message) {
    highPriorityQueue.push(message);
    pendingConfirmation = true;
    lastEndSend = millis();
    sendAttempts = 0;
}

void sendWithTDMA(const char* command, uint32_t freq, bool isHighPriority) {
    String msg = String(command);
    if(isHighPriority) {
        highPriorityQueue.push(msg);
    } else {
        lowPriorityQueue.push(msg);
    }
    
    if(isHighPriority && checkTxSlot()) {
        sendCommand(msg.c_str(), freq);
        highPriorityQueue.pop();
    }
}

// ================= CALLBACKS RADIO ===================
void OnTxDone(void) {
    lora_idle = true;
    setFrequency(FREQ_BASE);
    Radio.Rx(0);
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    
    // Verificar checksum
    char* sep = strrchr(rxpacket, ':');
    if(sep && strlen(sep) == 3) { // Checksum de 2 dígitos
        uint8_t rxChecksum = strtoul(sep+1, NULL, 16);
        *sep = '\0'; // Separar el checksum
        
        if(computeChecksum(rxpacket, strlen(rxpacket)) != rxChecksum) {
            Serial.println("Checksum inválido!");
            return;
        }
    }
    
    Serial.printf("Recibido [%s]: %s (RSSI: %d)\n", 
                 (deviceId == BASE_FIJA_ID) ? "869MHz" : "915MHz",
                 rxpacket, rssi);

    // Manejar token
    if(strncmp(rxpacket, "TOKEN:", 6) == 0) {
        hasToken = true;
        lastTokenActivity = millis();
        
        // Procesar transmisiones
        if(deviceId != BASE_FIJA_ID) {
            String telemetry = "TEL:" + String(deviceId) + ":" + readSensorData();
            sendCommand(telemetry.c_str(), FREQ_BASE);
        }
        
        passToken(); // Pasar token
    }
    // Manejar sincronización
    else if(strncmp(rxpacket, "SYNC", 4) == 0) {
        lastSync = millis(); // <--- Actualizar incluso en la base
        syncMissed = 0;
        Serial.printf("Sincronizado a %lu\n", lastSync);
    }
    // Resto de lógica existente...
else if(strncmp(rxpacket, "RTN:", 4) == 0) {
    if(deviceId != BASE_FIJA_ID) { // Solo remotos ejecutan
        currentRoutine = rxpacket + 4;
        routineInProgress = true;
        routineStartTime = millis();
        
        String ack = "ACK:" + String(deviceId) + ":" + currentRoutine;
        sendWithTDMA(ack.c_str(), FREQ_BASE);
    }
}
else if(strncmp(rxpacket, "END:", 4) == 0) {
    char* idPart = strchr(rxpacket, ':') + 1;
    int remoteId = atoi(idPart);
    
    if(deviceId == BASE_FIJA_ID) { // Solo base procesa
        if(remoteId > 0 && remoteId <= MAX_REMOTES) {
            remotes[remoteId-1].lastStatus = "Completado: " + String(strchr(idPart, ':') + 1);
            remotes[remoteId-1].lastUpdate = millis();
            
            // Enviar ACK al remoto original
            String ackMsg = "ACK_END:" + String(remoteId);
            sendCommand(ackMsg.c_str(), FREQ_REMOTOS);
        }
    } 
    else { // Reenviar en el anillo
        int targetNode = (deviceId + 1) % TOTAL_SLOTS;
        uint32_t freq = (targetNode == BASE_FIJA_ID) ? FREQ_BASE : FREQ_REMOTOS;
        sendCommand(rxpacket, freq);
    }
}
else if(strncmp(rxpacket, "ACK_END:", 8) == 0) {
    int ackedRemote = atoi(rxpacket + 8);
    if(ackedRemote == deviceId) {
        pendingConfirmation = false;
        highPriorityQueue.pop();
        Serial.println("ACK recibido - Rutina finalizada");
    }
}
// Manejar telemetría de remotos
else if(strncmp(rxpacket, "TEL:", 4) == 0) {
    // Extraer ID del remoto (después de "TEL:")
    int remoteId = atoi(rxpacket + 4);
    
    // Buscar la posición de los dos puntos para obtener los datos del sensor
    char* primerDosPuntos = strchr(rxpacket, ':');       // Primer ":" después de "TEL"
    char* segundoDosPuntos = strchr(primerDosPuntos + 1, ':');  // Segundo ":" después del ID
    
    if(remoteId > 0 && remoteId <= MAX_REMOTES && segundoDosPuntos) {
        char* datosSensor = segundoDosPuntos + 1; // Datos después del segundo ":"
        
        remotes[remoteId-1].id = remoteId;
        remotes[remoteId-1].lastUpdate = millis();
        remotes[remoteId-1].rssi = rssi;
        
        // Parsear los datos del sensor correctamente
        if(sscanf(datosSensor, "T:%f,H:%f,P:%f,A:%f,V:%f,B:%f",
            &remotes[remoteId-1].temperature,
            &remotes[remoteId-1].humidity,
            &remotes[remoteId-1].pressure,
            &remotes[remoteId-1].altitude,
            &remotes[remoteId-1].voltage,
            &remotes[remoteId-1].batteryPercent) != 6) {
            Serial.println("Error al parsear datos TEL");
        }
    } else {
        Serial.println("Formato TEL inválido");
    }
}
  

    else if(strncmp(rxpacket, "ACK:", 4) == 0) {
        waitingForAck = false;
    }
    
    lora_idle = true;
}


// ================= TOKEN RING ======================
void passToken() {
    int targetNode = (deviceId + 1) % TOTAL_SLOTS;
    uint32_t freq = (targetNode == BASE_FIJA_ID) ? FREQ_BASE : FREQ_REMOTOS;
    String tokenMsg = "TOKEN:" + String(deviceId);
    sendCommand(tokenMsg.c_str(), freq);
    hasToken = false;
    lastTokenActivity = millis();
}
// ================= FUNCIONES TDMA ====================
void checkTDMA() {
    if(deviceId == BASE_FIJA_ID) return;

    unsigned long currentTime = millis() - lastSync;
    unsigned long slotIndex = (currentTime / TDMA_SLOT_DURATION) % TOTAL_SLOTS;
    
    if(slotIndex == (unsigned long)deviceId) {
        // 1. Priorizar mensajes críticos
        if(!highPriorityQueue.empty()) {
            sendCommand(highPriorityQueue.front().c_str(), FREQ_BASE);
            highPriorityQueue.pop();
        }
        // 2. Enviar telemetría si no hay mensajes críticos
        else if(!lowPriorityQueue.empty()) {
            sendCommand(lowPriorityQueue.front().c_str(), FREQ_BASE);
            lowPriorityQueue.pop();
        }
        
        waitingForSlot = false;
    }
}
//========================= RUTINAS =======================================================
// ================= MOVIMIENTOS BÁSICOS =============
void moverAdelante() {
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
  // Patrón PWM: analogWrite(MOTOR_A1, 200); // Para control de velocidad
}

void moverAtras() {
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
}

void detenerMotores() {
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, LOW);
}

void giroDerecha(int tiempo) {
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, HIGH); // Frenado diferencial
    digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, HIGH);
  delay(tiempo);
  detenerMotores();
}

void giroIzquierda(int tiempo) {
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, LOW);  // Frenado diferencial
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, HIGH); 
  delay(tiempo);
  detenerMotores();
}

// ================= RUTINAS COMPLEJAS ===============
void rutinaExploracion() {
  for(int i = 0; i < 3; i++) {
    moverAdelante();
    delay(900);
  }
  detenerMotores();
}

void rutinaDance() {
  // Secuencia coreografiada
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_B1, HIGH);
  delay(200);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B2, HIGH);
  delay(200);
  for(int i=0; i<7; i++){
    moverAdelante();
    delay(700);
    moverAtras();
    delay(700);
  }
  giroIzquierda(500);
}

void rutinaEmergencia() {
  for(int i=0; i<7; i++){
    digitalWrite(MOTOR_A1, HIGH);
      digitalWrite(MOTOR_B1, HIGH);
    delay(700);
    digitalWrite(MOTOR_B2, HIGH);
      digitalWrite(MOTOR_B1, HIGH);
    delay(700);
  }
  detenerMotores();
}



//rutina
void executeCurrentRoutine() {
  String pinStatus = "";
  
  if(currentRoutine == "RUTINA_1") {
    rutinaExploracion();
    pinStatus = "Exploracion";
  } 
  else if(currentRoutine == "RUTINA_2") {
    rutinaDance();
    pinStatus = "Modo Dance";
  }
  else if(currentRoutine == "RUTINA_3") {
    rutinaEmergencia();
    pinStatus = "Emergencia";
  }
  unsigned long duration = millis() - routineStartTime;
  
  if(deviceId != BASE_FIJA_ID) {
    String remoteMsg = "REM:" + String(deviceId) + ":" + currentRoutine + ":DONE";
    sendWithTDMA(remoteMsg.c_str(), FREQ_REMOTOS);
  }
  
if(deviceId != BASE_FIJA_ID) {
    String endMsg = "END:" + String(deviceId) + ":" + currentRoutine;
    sendCriticalMessage(endMsg); // Usa cola de alta prioridad
}

    routineInProgress = false;
    updateDisplay("Rutina completada", currentRoutine, pinStatus);;

}

// ================= FUNCIONES BASE ====================
void processBaseFijaMode() {
    if(lora_idle) {
        Radio.Rx(0); // Escuchar constantemente
        lora_idle = false;
        Serial.println("Base en modo escucha...");
    }
}

// En processBaseRemotaMode()
void processBaseRemotaMode() {
    if(lora_idle) {
        setFrequency(FREQ_BASE); // Escuchar en frecuencia de remotos //cambio
        Radio.Rx(0);
        lora_idle = false;
    }
}

// ================= MANEJO DE ERRORES =================
void checkAckTimeout() {
    if(waitingForAck && (millis() - ackWaitStart > ACK_TIMEOUT)) {
        Serial.println("Timeout ACK, reintentando...");
        waitingForAck = false;
    }
}

// ================= SISTEMA ===========================
void printSystemInfo() {
    Serial.println("=== ESTADO DEL SISTEMA ===");
    Serial.printf("Dispositivo ID: %d\n", deviceId);
    Serial.printf("Slot TDMA actual: %lu\n", (millis() / TDMA_SLOT_DURATION) % TOTAL_SLOTS);
    Serial.println("=========================");
}

// ================= COMANDOS SERIAL ===================
void processSerialCommand(String command) {
  command.trim();
  
  if (command == "HELP") {
    printHelp();
  } 
  else if (command.startsWith("SET_ID=")) {
    int newId = command.substring(7).toInt();
    
    // Validar ID
    if (newId < 0 || newId >= TOTAL_SLOTS) {
      Serial.println("Error: ID debe estar entre 0 y " + String(TOTAL_SLOTS - 1));
      return;
    }
    deviceId = newId;
    saveDeviceId(); // Guardar en EEPROM
    // Detener operaciones en curso
    lora_idle = true;
    routineInProgress = false;
    waitingForAck = false;
    waitingForSlot = false;

    // Gestión del BME280
    if (newId != BASE_FIJA_ID) {
      // Modo remoto - Reinicializar BME280
      bme = Adafruit_BME280(); // Reset completo del objeto
      Wire2.begin(BME_SDA, BME_SCL);
      
      if (!bme.begin(BME_ADDRESS, &Wire2)) {
        Serial.println("Error: BME280 no detectado");
        deviceId = BASE_FIJA_ID; // Revertir a base por fallo
        return;
      }
      
      // Configuración avanzada del sensor
      bme.setSampling(
        Adafruit_BME280::MODE_NORMAL,
        Adafruit_BME280::SAMPLING_X2,
        Adafruit_BME280::SAMPLING_X16,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::FILTER_X16,
        Adafruit_BME280::STANDBY_MS_1000
      );
      Serial.println("BME280 configurado para modo remoto");
    } else {
      // Modo base - Liberar recursos del sensor
      bme = Adafruit_BME280(); // Destruir instancia
      Serial.println("Modo base - Sensor desactivado");
    }

    // Actualizar ID y frecuencia
    deviceId = newId;
    uint32_t newFreq = (deviceId == BASE_FIJA_ID) ? FREQ_BASE : FREQ_REMOTOS;
    
    // Reinicializar radio
    Radio.Init(&RadioEvents);
    setFrequency(newFreq);
    Radio.Rx(0);

    // Actualizar periféricos
    updateDisplay("ID Cambiado:", "Nuevo ID: " + String(deviceId), "");
    Serial.println("ID actualizado: " + String(deviceId) + " | Frecuencia: " + String(newFreq / 1e6) + " MHz");
  }
else if (command.startsWith("START_ROUTINE=") && deviceId == BASE_FIJA_ID) {
    String routine = command.substring(14);
    if (routine.length() > 0) {
        String commandToSend;
        if(routine == "1") commandToSend = "RTN:RUTINA_1";
        else if(routine == "2") commandToSend = "RTN:RUTINA_2";
        else if(routine == "3") commandToSend = "RTN:RUTINA_3";
        else commandToSend = "RTN:STOP";
        
        sendCommand(commandToSend.c_str(), FREQ_BASE);
        waitingForAck = true;
        ackWaitStart = millis();
        updateDisplay("Comando enviado", commandToSend, "");
    }
}
  else {
    Serial.println("Comando no reconocido. Escribe HELP para ayuda.");
  }
}
// ================= SAFING MODE =====================
void activateSafing() {
      systemStatus = "Modo Seguro Activado";
    detenerMotores();
    inSafing = true;
    lora_idle = true;
    Radio.Sleep();
    Serial.println("MODO SEGURO ACTIVADO!");
    updateDisplay("MODO SEGURO", "Comunicaciones perdidas", "");
}
void printHelp() {
    Serial.println("\n=== COMANDOS DISPONIBLES ===");
    Serial.println("SET_ID=X - Cambiar ID dispositivo (0-2)");
    Serial.println("START_ROUTINE=<nombre> - Iniciar rutina (solo base)");
    Serial.println("HELP - Mostrar ayuda");
    Serial.println("============================");
}

// ================= CONFIGURACIÓN LoRa ================
void setFrequency(uint32_t freq) {
    Radio.SetChannel(freq);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
}


void setupWebServer() {
    WiFi.softAPConfig(localIP, gateway, subnet);
    WiFi.softAP(apSSID, apPassword);

    server.on("/", HTTP_GET, []() {
        String html = R"=====(
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Control Base LoRa</title>
    <style>
        :root {
            --primary: #2c3e50;
            --secondary: #3498db;
            --success: #27ae60;
            --danger: #e74c3c;
            --text: #34495e;
        }

        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            padding: 20px;
            background: #ecf0f1;
            color: var(--text);
        }

        .header {
            background: white;
            padding: 2rem;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            margin-bottom: 2rem;
        }

        .grid-container {
            display: grid;
            gap: 1.5rem;
            grid-template-columns: repeat(auto-fill, minmax(320px, 1fr));
        }

        .node-card {
            background: white;
            border-radius: 10px;
            padding: 1.5rem;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            border-left: 4px solid;
        }

        .node-card.offline {
            border-color: var(--danger);
            opacity: 0.7;
        }

        .node-card.online {
            border-color: var(--success);
        }

        .node-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 1rem;
        }

        .node-title {
            font-size: 1.2rem;
            font-weight: 600;
        }

        .node-status {
            display: flex;
            align-items: center;
            gap: 0.5rem;
        }

        .status-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
        }

        .online-dot { background: var(--success); }
        .offline-dot { background: var(--danger); }

        .sensor-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 1rem;
            margin: 1rem 0;
        }

        .sensor-item {
            background: #f8f9fa;
            padding: 1rem;
            border-radius: 8px;
            text-align: center;
        }

        .sensor-value {
            font-size: 1.4rem;
            font-weight: 600;
            margin: 0.5rem 0;
        }

        .sensor-unit {
            color: #7f8c8d;
            font-size: 0.9rem;
        }

        .routine-status {
            margin-top: 1rem;
            padding: 1rem;
            background: #f8f9fa;
            border-radius: 8px;
            text-align: center;
        }

        .routine-active {
            background: #e3f2fd;
            color: #1976d2;
        }

        .button-group {
            display: flex;
            gap: 0.5rem;
            margin-top: 1rem;
        }

        button {
            flex: 1;
            padding: 0.8rem;
            border: none;
            border-radius: 6px;
            cursor: pointer;
            font-weight: 500;
            transition: all 0.2s;
        }

        .btn-primary {
            background: var(--secondary);
            color: white;
        }

        .btn-danger {
            background: var(--danger);
            color: white;
        }

        button:hover {
            opacity: 0.9;
            transform: translateY(-1px);
        }
    </style>
        <style>
        
        .control-panel {
            margin-bottom: 2rem;
            background: white;
            padding: 1.5rem;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        
        .routine-buttons {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 1rem;
            margin-top: 1rem;
        }
        
        .btn-routine {
            background: var(--success);
        }
        
        .btn-stop {
            background: var(--danger);
        }
    </style>
</head>
<body>
 <div class="header">
        <h1>Control de Dispositivos LoRa</h1>
        <div class="system-status">
            <p>Estado General: <strong id="systemStatus">${systemStatus}</strong></p>
            <p>Rutina Activa: <strong id="currentRoutine">${currentRoutine}</strong></p>
        </div>
        
        <!-- Añade este panel de control -->
        <div class="control-panel">
            <h3>Control de Rutinas</h3>
            <div class="routine-buttons">
                <button class="btn-routine" onclick="controlRoutine(1)">Rutina 1</button>
                <button class="btn-routine" onclick="controlRoutine(2)">Rutina 2</button>
                <button class="btn-routine" onclick="controlRoutine(3)">Rutina 3</button>
                <button class="btn-stop" onclick="controlRoutine(0)">🛑 Detener</button>
            </div>
        </div>
    </div>

    <div class="grid-container" id="nodesContainer">
        <!-- Tarjetas dinámicas de nodos -->
    </div>

    <script>
        function updateData() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('systemStatus').textContent = data.systemStatus;
                    document.getElementById('currentRoutine').textContent = data.currentRoutine;

                    const nodesHtml = data.remotes.map(remote => `
                        <div class="node-card ${remote.lastUpdate > 60 ? 'offline' : 'online'}">
                            <div class="node-header">
                                <div class="node-title">
                                    📶 Dispositivo #${remote.id}
                                </div>
                                <div class="node-status">
                                    <div class="status-dot ${remote.lastUpdate > 60 ? 'offline-dot' : 'online-dot'}"></div>
                                    <span>${remote.lastUpdate > 60 ? 'Offline' : 'Online'}</span>
                                </div>
                            </div>

                            <div class="sensor-grid">
                                <div class="sensor-item">
                                    <div class="sensor-label">🌡 Temperatura</div>
                                    <div class="sensor-value">${remote.temperature?.toFixed(1) || '--'}</div>
                                    <div class="sensor-unit">°C</div>
                                </div>
                                
                                <div class="sensor-item">
                                    <div class="sensor-label">💧 Humedad</div>
                                    <div class="sensor-value">${remote.humidity?.toFixed(1) || '--'}</div>
                                    <div class="sensor-unit">%</div>
                                </div>
                                
                                <div class="sensor-item">
                                    <div class="sensor-label">🎚 Presión</div>
                                    <div class="sensor-value">${remote.pressure?.toFixed(1) || '--'}</div>
                                    <div class="sensor-unit">hPa</div>
                                </div>
                                
                                <div class="sensor-item">
                                    <div class="sensor-label">📏 Altitud</div>
                                    <div class="sensor-value">${remote.altitude?.toFixed(1) || '--'}</div>
                                    <div class="sensor-unit">metros</div>
                                </div>
                            </div>

                            <div class="routine-status ${remote.lastStatus === 'Completado' ? 'routine-active' : ''}">
                                ${remote.lastStatus === 'Completado' ? 
                                    `✅ Rutina #${remote.lastStatus} completada` : 
                                    `⏳ Última rutina: #${remote.lastStatus || '--'}`
                                }
                            </div>

                            <div class="battery-status">
                                <div class="sensor-label">🔋 Batería</div>
                                <div class="sensor-value">${remote.voltage?.toFixed(2) || '--'}V</div>
                                <div class="sensor-unit">(${remote.batteryPercent?.toFixed(0) || '--'}%)</div>
                            </div>
                        </div>
                    `).join('');

                    document.getElementById('nodesContainer').innerHTML = nodesHtml;
                });
        }

        function controlRoutine(routineId) {
            fetch(`/control?routine=${routineId}`)
                .then(() => updateData());
        }

        setInterval(updateData, 2000);
        updateData();
    </script>
</body>
</html>
        )=====";
        server.send(200, "text/html", html);
    });

 server.on("/data", HTTP_GET, []() {
    String json = "{";
    json += "\"systemStatus\":\"" + systemStatus + "\",";
    json += "\"currentRoutine\":\"" + currentRoutine + "\",";
    json += "\"remotes\":[";
    for(int i=0; i<MAX_REMOTES; i++) {
        if(remotes[i].id != 0) {
            json += "{";
            json += "\"id\":" + String(remotes[i].id) + ",";
            json += "\"status\":\"" + remotes[i].lastStatus + "\",";
            json += "\"temperature\":" + String(remotes[i].temperature) + ",";
            json += "\"humidity\":" + String(remotes[i].humidity) + ",";
            json += "\"pressure\":" + String(remotes[i].pressure) + ",";
            json += "\"altitude\":" + String(remotes[i].altitude) + ",";
            json += "\"voltage\":" + String(remotes[i].voltage) + ",";
            json += "\"batteryPercent\":" + String(remotes[i].batteryPercent) + ",";
            json += "\"rssi\":" + String(remotes[i].rssi) + ",";
            json += "\"lastUpdate\":" + String((millis() - remotes[i].lastUpdate)/1000);
            json += "},";
        }
    }
        json = json.length() > 10 ? json.substring(0, json.length()-1) : json;
        json += "]}";
        server.send(200, "application/json", json);
    });

server.on("/control", HTTP_GET, []() {
    int routine = server.arg("routine").toInt();
    String routineName = "NINGUNA";
    
    switch(routine) {
        case 1: 
            routineName = "RUTINA_1";
            systemStatus = "Ejecutando Exploración";
            break;
        case 2: 
            routineName = "RUTINA_2";
            systemStatus = "Ejecutando Modo Dance";
            break;
        case 3: 
            routineName = "RUTINA_3";
            systemStatus = "Emergencia Activada";
            break;
        default:
            systemStatus = "Sistema Detenido";
    }

    if(routine > 0 && routine <= 3) {
        currentRoutine = routineName;
        String command = "RTN:" + String(routineName);
        sendCommand(command.c_str(), FREQ_BASE);
    } else {
        currentRoutine = "NINGUNA";
        sendCommand("RTN:STOP", FREQ_BASE);
    }
    
    server.send(200, "text/plain", "OK");
});

    server.begin();
}