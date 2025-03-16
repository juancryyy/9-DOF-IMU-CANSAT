% Eliminar conexiones anteriores
delete(instrfind('Port', 'COM3'));

% Configurar el puerto serial
s = serialport('COM3', 115200);

% Calibrar giroscopio, acelerómetro y magnetómetro
ref_data = CalibracionSensor(s);

% Definir el número total de muestras a recibir
num_muestras = 1000; 

% Inicializar matrices para giroscopio, acelerómetro y magnetómetro
gyro = zeros(num_muestras, 3); 
acelero = zeros(num_muestras, 3); 
magneto = zeros(num_muestras, 3); 
quaternion = [1 0 0 0]; % Cuaternión inicial


% Parámetros del filtro
beta = 0.1; 
sampleFreq = 1/10; % Frecuencia de muestreo (100 Hz)

% Crear una figura para visualización
figure;

for i = 1:num_muestras
    % Esperar a que haya datos disponibles
    while s.NumBytesAvailable == 0
        pause(0.01);
    end
    
    % Leer datos del sensor
    data = readline(s);
    dataArray = str2double(strsplit(data, ','));
    
    if length(dataArray) == 9
        % Aplicar la calibración restando los valores de referencia para cada posición
        pos_idx = determinePosition(dataArray, ref_data); % Función para determinar la posición actual
        gyro(i, :) = dataArray(1:3) - ref_data(pos_idx).gyro.mean;   % Restar la referencia del giroscopio
        acelero(i, :) = dataArray(4:6) - ref_data(pos_idx).accel.mean; % Restar la referencia del acelerómetro
        magneto(i, 1) = dataArray(7) - ref_data(pos_idx).mag.mean(1); % Restar la referencia del magnetómetro
        magneto(i, 2) = dataArray(8) - ref_data(pos_idx).mag.mean(2);
        magneto(i, 3) = dataArray(9) - ref_data(pos_idx).mag.mean(3);

        % Actualizar el filtro Madgwick
        quaternion = madgwickAHRS(gyro(i, 1), gyro(i, 2), gyro(i, 3), ...
                                   acelero(i, 1), acelero(i, 2), acelero(i, 3), ...
                                   magneto(i, 1), magneto(i, 2), magneto(i, 3), ...
                                   quaternion, beta, sampleFreq);
        
        % Visualización
        plotBlock(quaternion); 

        drawnow; % Actualiza la figura
    end

    pause(0.01); % Pausa para permitir respuesta
end

% Cerrar y limpiar el puerto serial
fclose(s);
delete(s);
clear s;
