function pos_idx = determinePosition(dataArray, ref_data)
    gyro = dataArray(1:3);
    
    for pos = 1:length(ref_data)
        gyro_range = ref_data(pos).gyro.range;
        % Verifica si los valores de giroscopio están dentro del rango
        if all(gyro >= gyro_range(1, :) & gyro <= gyro_range(2, :))
            pos_idx = pos; % Retorna el índice de la posición válida
            return;
        end
    end
    pos_idx = 1; % Valor por defecto si no se encuentra la posición
end
