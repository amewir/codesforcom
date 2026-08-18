function [img_gris, img_infrarojo] = procesar_imagen(imagen_filename, imagen_filename2, margen)

    if nargin < 3
        margen = 30; % Margen más grande por defecto
    end

    % Cargar la imagen original
    imgLe = imread(imagen_filename);
    if isempty(imgLe)
        error('No se pudo cargar la imagen. Verifica el nombre del archivo.');
    end

    % Cargar la segunda imagen
    imgLe2 = imread(imagen_filename2);
    if isempty(imgLe2)
        error('No se pudo cargar la imagen. Verifica el nombre del archivo.');
    end

    % Obtener el tamaño de las imágenes
    [altura, ancho, ~] = size(imgLe);
    [altura2, ancho2, ~] = size(imgLe2);

    % Crear fondo negro con márgenes
    fondo_negro = zeros(altura + 2*margen, ancho + 2*margen, 3, 'uint8');

    % Superponer la primera imagen sobre el fondo negro
    if size(imgLe, 3) == 4 % Si es RGBA
        alpha_channel = imgLe(:,:,4) / 255; % Normalizar el canal alfa
        for i = 1:3
            fondo_negro(margen+1:margen+altura, margen+1:margen+ancho, i) = ...
                uint8(double(imgLe(:,:,i)) .* alpha_channel + double(fondo_negro(margen+1:margen+altura, margen+1:margen+ancho, i)) .* (1 - alpha_channel));
        end
    else
        fondo_negro(margen+1:margen+altura, margen+1:margen+ancho, :) = imgLe; % Usar la imagen directamente
    end

    % Convertir la imagen con fondo negro a escala de grises
    img_gris = rgb2gray(fondo_negro);
    img_gris = uint8(double(img_gris) * 255 / double(max(img_gris(:)))); % Normalizar a 8 bits

    % Crear fondo negro para la segunda imagen con márgenes
    fondo_negro2 = zeros(altura2 + 2*margen, ancho2 + 2*margen, 3, 'uint8');

    % Superponer la segunda imagen sobre el fondo negro
    if size(imgLe2, 3) == 4 % Si es RGBA
        alpha_channel2 = imgLe2(:,:,4) / 255; % Normalizar el canal alfa
        for i = 1:3
            fondo_negro2(margen+1:margen+altura2, margen+1:margen+ancho2, i) = ...
                uint8(double(imgLe2(:,:,i)) .* alpha_channel2 + double(fondo_negro2(margen+1:margen+altura2, margen+1:margen+ancho2, i)) .* (1 - alpha_channel2));
        end
    else
        fondo_negro2(margen+1:margen+altura2, margen+1:margen+ancho2, :) = imgLe2; % Usar la imagen directamente
    end

    % Convertir la segunda imagen a escala de grises y simular infrarrojo
    img_infrarojo = rgb2gray(fondo_negro2);
    img_infrarojo = imadjust(img_infrarojo, [0.3 0.7], []); % Ajustar contraste para simular infrarrojo

    % Guardar las imágenes procesadas
    imwrite(img_gris, 'imagen_escala_grises_final.jpg');
    imwrite(img_infrarojo, 'imagen_infraroja_final.jpg');

    % Mostrar resultados
    figure;
    subplot(1, 2, 1);
    imshow(img_gris);
    title('Imagen en Escala de Grises');

    subplot(1, 2, 2);
    imshow(img_infrarojo);
    title('Imagen Simulada en Infrarrojo');
end