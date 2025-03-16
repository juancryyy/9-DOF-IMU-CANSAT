function [ref_data] = CalibracionSensor(s)
    % Definir las posiciones y tiempos de calibración
    posiciones = {'horizontal', 'boca abajo', 'hacia adelante', 'hacia atrás', 'derecha', 'izquierda'};
    tiempo_calibracion = 1; % Tiempo en segundos para la recolección de muestras
    num_calibracion = 100; % Número de muestras por posición

    % Inicializar estructuras para almacenar referencias
    ref_data = struct('gyro', [], 'accel', [], 'mag', []);

    for pos = 1:length(posiciones)
        fprintf('Mantener el sensor en posición %s durante %d segundos...\n', posiciones{pos}, tiempo_calibracion);
        pause(tiempo_calibracion); % Esperar para indicar al usuario que mantenga la posición

        % Inicializar matrices para cada posición
        gyro_ref = zeros(num_calibracion, 3);
        accel_ref = zeros(num_calibracion, 3);
        mag_ref = zeros(num_calibracion, 3);
        
        % Recolección de datos para la calibración
        for i = 1:num_calibracion
            while s.NumBytesAvailable == 0
                pause(0.01);
            end
            
            data = readline(s);
            dataArray = str2double(strsplit(data, ','));
            
            if length(dataArray) == 9
                gyro_ref(i, :) = dataArray(1:3);  % Giroscopio
                accel_ref(i, :) = dataArray(4:6); % Acelerómetro
                mag_ref(i, 1) = dataArray(7);      % Magnetómetro X
                mag_ref(i, 2) = dataArray(8);      % Magnetómetro Y
                mag_ref(i, 3) = dataArray(9);      % Magnetómetro Z
            end

            pause(0.01); % Pequeña pausa para evitar saturar la lectura
        end

        % Calcular el promedio y el rango
        gyro_avg = mean(gyro_ref, 1);
        accel_avg = mean(accel_ref, 1);
        mag_avg = mean(mag_ref, 1);
        
        gyro_range = [min(gyro_ref); max(gyro_ref)];
        accel_range = [min(accel_ref); max(accel_ref)];
        mag_range = [min(mag_ref); max(mag_ref)];

        % Almacenar datos de referencia
        ref_data(pos).gyro.mean = gyro_avg;
        ref_data(pos).gyro.range = gyro_range;
        ref_data(pos).accel.mean = accel_avg;
        ref_data(pos).accel.range = accel_range;
        ref_data(pos).mag.mean = mag_avg;
        ref_data(pos).mag.range = mag_range;

        fprintf('Valores de calibración para %s:\n', posiciones{pos});
        fprintf('Giroscopio promedio: [%f %f %f], Rango: [%f %f; %f %f; %f %f]\n', ...
            gyro_avg, gyro_range(1,1), gyro_range(2,1), gyro_range(1,2), gyro_range(2,2), gyro_range(1,3), gyro_range(2,3));
        fprintf('Acelerómetro promedio: [%f %f %f], Rango: [%f %f; %f %f; %f %f]\n', ...
            accel_avg, accel_range(1,1), accel_range(2,1), accel_range(1,2), accel_range(2,2), accel_range(1,3), accel_range(2,3));
        fprintf('Magnetómetro promedio: [%f %f %f], Rango: [%f %f; %f %f; %f %f]\n', ...
            mag_avg, mag_range(1,1), mag_range(2,1), mag_range(1,2), mag_range(2,2), mag_range(1,3), mag_range(2,3));

        % Indicar que ha terminado esta posición y esperar un momento
        pause(2);
    end
end
