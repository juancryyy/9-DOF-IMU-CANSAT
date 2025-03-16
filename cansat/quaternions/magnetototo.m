% Parámetros para la conexión serial
serial_port = 'COM3'; % Cambiar a tu puerto serial
baud_rate = 115200;   % Cambiar si es necesario
serial_obj = serialport(serial_port, baud_rate);

% Variables para almacenamiento de datos
data_points = 500; % Número de muestras a tomar
mag_data = zeros(data_points, 3); % Almacena x, y, z del magnetómetro

% Lectura de datos seriales
disp('Iniciando la captura de datos...');

for i = 1:data_points
    raw_data = readline(serial_obj);  % Leer línea de datos
    split_data = str2double(strsplit(raw_data, ','));  % Separar por comas y convertir a números
    mag_data(i, :) = split_data(1:3);  % Guardar valores de x, y, z
end

%% Plotear los datos crudos con proyecciones XY, XZ, YZ
figure;
hold on;
scatter(mag_data(:, 1), mag_data(:, 2), 'filled'); % XY
scatter(mag_data(:, 1), mag_data(:, 3), 'filled'); % XZ
scatter(mag_data(:, 2), mag_data(:, 3), 'filled'); % YZ
hold off;
title('Proyecciones crudas en planos XY, XZ, YZ');
xlabel('X');
ylabel('Y / Z');
legend('XY', 'XZ', 'YZ');
grid on;

%% Calibración de Hard Iron (Corrección de Offset)
offset_x = (max(mag_data(:, 1)) + min(mag_data(:, 1))) / 2;
offset_y = (max(mag_data(:, 2)) + min(mag_data(:, 2))) / 2;
offset_z = (max(mag_data(:, 3)) + min(mag_data(:, 3))) / 2;

% Aplicar la corrección de hard iron
corrected_data_hard_iron = mag_data - [offset_x, offset_y, offset_z];

%% Plotear datos corregidos por Hard Iron
figure;
hold on;
scatter(corrected_data_hard_iron(:, 1), corrected_data_hard_iron(:, 2), 'filled'); % XY
scatter(corrected_data_hard_iron(:, 1), corrected_data_hard_iron(:, 3), 'filled'); % XZ
scatter(corrected_data_hard_iron(:, 2), corrected_data_hard_iron(:, 3), 'filled'); % YZ
hold off;
title('Proyecciones corregidas (Hard Iron) en planos XY, XZ, YZ');
xlabel('X');
ylabel('Y / Z');
legend('XY', 'XZ', 'YZ');
grid on;

%% Calibración de Soft Iron (Corrección de Escala)
avg_delta_x = (max(corrected_data_hard_iron(:, 1)) - min(corrected_data_hard_iron(:, 1))) / 2;
avg_delta_y = (max(corrected_data_hard_iron(:, 2)) - min(corrected_data_hard_iron(:, 2))) / 2;
avg_delta_z = (max(corrected_data_hard_iron(:, 3)) - min(corrected_data_hard_iron(:, 3))) / 2;

avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3;

scale_x = avg_delta / avg_delta_x;
scale_y = avg_delta / avg_delta_y;
scale_z = avg_delta / avg_delta_z;

% Aplicar la corrección de soft iron
corrected_data_soft_iron = corrected_data_hard_iron .* [scale_x, scale_y, scale_z];

%% Plotear datos corregidos por Soft Iron con proyecciones XY, XZ, YZ
figure;
hold on;
scatter(corrected_data_soft_iron(:, 1), corrected_data_soft_iron(:, 2), 'filled'); % XY
scatter(corrected_data_soft_iron(:, 1), corrected_data_soft_iron(:, 3), 'filled'); % XZ
scatter(corrected_data_soft_iron(:, 2), corrected_data_soft_iron(:, 3), 'filled'); % YZ
hold off;
title('Proyecciones corregidas (Soft Iron) en planos XY, XZ, YZ');
xlabel('X');
ylabel('Y / Z');
legend('XY', 'XZ', 'YZ');
grid on;

%% Cerrar la conexión serial
clear serial_obj;
disp('Proceso de calibración completado.');

