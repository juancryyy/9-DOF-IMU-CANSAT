function [x_calib, y_calib, z_calib] = CalibracionMagneto(s, num_muestras)
    % Inicializar el almacenamiento de datos
    magneto = zeros(num_muestras, 3); 
    tic; % Iniciar temporizador
    i = 1;

    % Recoger datos del magnetómetro durante 10 segundos
    while toc < 10 && i <= num_muestras
        if s.NumBytesAvailable > 0
            data = readline(s);
            dataArray = str2double(strsplit(data, ','));

            if length(dataArray) == 9
                magneto(i, :) = dataArray(7:9); % Almacenar datos de magnetómetro
                i = i + 1;
            end
        end
    end

    % Calcular los valores de calibración
    x_calib = mean(magneto(:, 1)); % Valor promedio de X
    y_calib = mean(magneto(:, 2)); % Valor promedio de Y
    z_calib = mean(magneto(:, 3)); % Valor promedio de Z

    disp('Calibración completada:');
    fprintf('X_calib: %.2f, Y_calib: %.2f, Z_calib: %.2f\n', x_calib, y_calib, z_calib);
end

