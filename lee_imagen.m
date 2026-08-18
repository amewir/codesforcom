function [apt_signal] = lee_imagen(imagen_leida, imagen_leida2)

    % Cargar las imágenes
    imgA = imread(imagen_leida);   % Imagen para Canal A (Visible)
    imgB = imread(imagen_leida2);  % Imagen para Canal B (Infrarrojo)

    % Redimensionar a un tamaño estándar (400 líneas por 480 píxeles)
    imgA = imresize(imgA, [400, 909]);
    imgB = imresize(imgB, [400, 909]);

    % Normalizar las imágenes a [0, 1]
    imgA = double(imgA) / 255;  % Normalización de Canal A
    imgB = double(imgB) / 255;  % Normalización de Canal B

    % Crear una imagen combinada apilando horizontalmente
    combinada = [imgA, imgB];  % Apilar horizontalmente

    % Guardar la imagen combinada
    imwrite(combinada, 'imagen_combinada.jpg');

    % Parámetros de transmisión
    fs = 11025;  % Frecuencia de muestreo estándar para NOAA APT
    t_sync = (0:1/fs:0.0016)';  % Duración de sincronización (~16ms), vector columna
    t_line = (0:1/fs:0.5)';   % Duración de cada línea (~0.5 segundos)

    % Frecuencias de sincronización para A y B
    syncA_freq = 1040;  % Frecuencia de sincronización para el Canal A
    syncB_freq = 832;   % Frecuencia de sincronización para el Canal B

    % Crear las señales de sincronización A y B
    syncA = square(2 * pi * syncA_freq * t_sync);  % Onda cuadrada para Sync A
    pulse_duration = round(0.001 * fs);  % Duración de cada pulso (~1ms)
    pulse_train = zeros(length(t_sync), 1);  % Inicializar señal de tren de pulsos
    pulse_train(1:pulse_duration:end) = 1;   % Insertar pulsos en la señal
    
    syncB = pulse_train;  % Asignar el tren de pulsos a Sync B

    % *1. 3 segundos de señal de inicio*
    start_signal = 0.5 * ones(3 * fs, 1);  % Señal constante para los primeros 3 segundos

    % *2. 5 segundos de señal de phasing*
    phasing_signal = repmat([syncA; syncB], ceil(5 * fs / length(syncA)), 1);

    % Inicializar la señal combinada APT con el inicio y el phasing
    apt_signal = [start_signal; phasing_signal];

    % *3. Transmisión de 400 líneas de imagen en los Canales A y B*
    for i = 1:size(combinada, 1)
        % Extraer la fila actual de la imagen combinada
        fila_combined = combinada(i, :)';  % Fila de la imagen combinada

        % Interpolar la fila para ajustarse al número de muestras (0.5 segundos por línea)
        num_samples = fs * 0.5;  % Número de muestras por fila (~0.5 segundos)
        fila_combined_mod = interp1(linspace(0, 1, numel(fila_combined)), fila_combined, linspace(0, 1, num_samples), 'linear')';

        % Ajustar el tiempo para corregir el slant
        slant_correction_factor = 1 + (i - 1) * 0.0001;  % Aumentar ligeramente para cada línea
        t_mod = (0:1/fs:(num_samples-1)/fs)' * slant_correction_factor;  % Aplicar la corrección de tiempo
        modulated_combined = cos(2 * pi * 2400 * t_mod) .* fila_combined_mod;  % Modulación para imagen combinada

        % Concatenar la sincronización A, modulación de la imagen combinada, y sincronización B
        linea_apt = [syncA; modulated_combined; syncB];

        % Añadir la línea completa a la señal APT
        apt_signal = [apt_signal; linea_apt];
    end

    % Guardar la señal combinada como archivo de audio
    audiowrite('apt_transmission_final_imagen.wav', apt_signal, fs);

end
