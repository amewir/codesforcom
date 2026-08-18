# Com 2 Spheres & APT Images

Este repositorio contiene código enfocado en telecomunicaciones, dividido en dos áreas principales:

## 1. Sistema de Comunicación LoRa para Esferas (Robótica)
El directorio `com2_spheres` contiene el firmware para un módulo ESP32/Heltec LoRa V3. Implementa una red de sensores y actuadores con las siguientes características:

- **Protocolo de Acceso Múltiple:** Utiliza TDMA (Time Division Multiple Access) con slots de 3 segundos para organizar la comunicación entre una Base Fija (ID 0) y nodos Remotos (ID 1, 2). También implementa una lógica de Token Ring para el paso del turno de transmisión.
- **División de Frecuencias (FDMA):** La Base Fija opera en 869 MHz, mientras que los Remotos se comunican entre sí en 915 MHz.
- **Telemetría:** Lee datos ambientales usando un sensor BME280 (Temperatura, Humedad, Presión, Altitud) y lee el voltaje de la batería, transmitiendo esta información a la base.
- **Control Remoto y Rutinas:** Recibe comandos para ejecutar rutinas de movimiento en motores (como `rutinaExploracion`, `rutinaDance`), controlando pines específicos.
- **Servidor Web:** La base despliega un servidor WiFi local para visualizar la telemetría recibida de los remotos.

## 2. Procesamiento de Imágenes APT (Automatic Picture Transmission)
Contiene scripts en MATLAB para simular o procesar señales de satélites meteorológicos NOAA:

- **`lee_imagen.m`**: Toma dos imágenes (Visible e Infrarrojo), las combina, y las modula sobre una subportadora de 2400 Hz. Genera los tonos de sincronización estándar de APT (1040 Hz para el canal A y 832 Hz para el canal B) a una frecuencia de muestreo de 11025 Hz, exportando el resultado como un archivo de audio WAV.
- **`procesar_imagen.m`**: Procesa las imágenes aplicándoles bordes negros, convirtiéndolas a escala de grises y simulando un canal infrarrojo mediante el ajuste de contrastes.


