function q = madgwickAHRS(gx, gy, gz, ax, ay, az, mx, my, mz, q, beta, sampleFreq)
    % Normalizar las lecturas del acelerómetro
    recipNorm = sqrt(ax * ax + ay * ay + az * az);
    if recipNorm == 0
        return; % Manejar NaN
    end
    ax = ax / recipNorm;
    ay = ay / recipNorm;
    az = az / recipNorm;
    
    % Normalizar las lecturas del magnetómetro
    recipNorm = sqrt(mx * mx + my * my + mz * mz);
    if recipNorm == 0
        return; % Manejar NaN
    end
    mx = mx / recipNorm;
    my = my / recipNorm;
    mz = mz / recipNorm;
    
    % Calcular la tasa de cambio del cuaternión
    qDot(1) = 0.5 * (-q(2) * gx - q(3) * gy - q(4) * gz);
    qDot(2) = 0.5 * (q(1) * gx + q(3) * gz - q(4) * gy);
    qDot(3) = 0.5 * (q(1) * gy - q(2) * gz + q(4) * gx);
    qDot(4) = 0.5 * (q(1) * gz + q(2) * gy - q(3) * gx);
    
    % Solo calcular el feedback si hay datos de acelerómetro válidos
    if ~(ax == 0 && ay == 0 && az == 0)
        % Normalizar medición del acelerómetro
        recipNorm = invSqrt(ax^2 + ay^2 + az^2);
        ax = ax * recipNorm;
        ay = ay * recipNorm;
        az = az * recipNorm;

        % Normalizar medición del magnetómetro
        recipNorm = invSqrt(mx^2 + my^2 + mz^2);
        mx = mx * recipNorm;
        my = my * recipNorm;
        mz = mz * recipNorm;
        
        % Calcular la dirección de referencia del campo magnético
        hx = mx * q(1)^2 - 2 * q(1) * my * q(4) + 2 * q(1) * mz * q(3) + mx * q(2)^2;
        hy = 2 * q(1) * mx * q(4) + my * q(1)^2 - 2 * q(1) * mz * q(2) + 2 * q(2) * mx * q(3);
        two_bx = sqrt(hx^2 + hy^2);
        two_bz = -2 * q(1) * mx * q(3) + 2 * q(1) * my * q(2) + mz * q(1)^2;
        
        % Aplicar paso correctivo
        s(1) = -2 * q(3) * (2 * q(2) * q(4) - 2 * q(1) * ax) + 2 * q(2) * (2 * q(1) * q(2) + 2 * q(3) * ay);
        s(2) = 2 * q(4) * (2 * q(2) * q(4) - 2 * q(1) * ax) + 2 * q(1) * (2 * q(1) * q(2) + 2 * q(3) * ay);
        s(3) = -2 * q(1) * (2 * q(2) * q(4) - 2 * q(1) * ax) + 2 * q(4) * (2 * q(1) * q(2) + 2 * q(3) * ay);
        s(4) = 2 * q(2) * (2 * q(2) * q(4) - 2 * q(1) * ax) + 2 * q(3) * (2 * q(1) * q(2) + 2 * q(3) * ay);

        recipNorm = invSqrt(s(1)^2 + s(2)^2 + s(3)^2 + s(4)^2);
        s = s * recipNorm;

        % Aplicar el paso correctivo al cuaternión
        qDot = qDot - beta * s;
    end
    
    % Integrar la tasa de cambio del cuaternión
    q = q + qDot * (1.0 / sampleFreq);

    % Normalizar el cuaternión
    recipNorm = invSqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);
    q = q * recipNorm;
end

% Función auxiliar para calcular la raíz inversa
function y = invSqrt(x)
    y = 1 / sqrt(x);
end
