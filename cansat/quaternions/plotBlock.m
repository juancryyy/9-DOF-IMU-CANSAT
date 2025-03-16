function plotBlock(quaternion)
    % Limpiar el contenido del gráfico antes de dibujar
    cla;

    % Definir las dimensiones del cuboide (largo, ancho, altura)
    dimensions = [2, 1, 3]; % Cambiar según las proporciones deseadas (Largo, Ancho, Altura)

    % Definir los vértices del cuboide
    vertices = [
        -dimensions(1)/2, -dimensions(2)/2, -dimensions(3)/2;
         dimensions(1)/2, -dimensions(2)/2, -dimensions(3)/2;
         dimensions(1)/2,  dimensions(2)/2, -dimensions(3)/2;
        -dimensions(1)/2,  dimensions(2)/2, -dimensions(3)/2;
        -dimensions(1)/2, -dimensions(2)/2,  dimensions(3)/2;
         dimensions(1)/2, -dimensions(2)/2,  dimensions(3)/2;
         dimensions(1)/2,  dimensions(2)/2,  dimensions(3)/2;
        -dimensions(1)/2,  dimensions(2)/2,  dimensions(3)/2
    ];

    % Definir las caras del cuboide
    faces = [
        1 2 3 4; % Cara inferior
        5 6 7 8; % Cara superior
        1 2 6 5; % Cara lateral 1
        2 3 7 6; % Cara lateral 2
        3 4 8 7; % Cara lateral 3
        4 1 5 8  % Cara lateral 4
    ];

    % Asignar colores a las caras (usando colores representativos de los ejes i, j, k)
    faceColors = [
        0 0 1;  % Azul para el eje Z (cara superior)
        1 0 0;  % Rojo para el eje X
        0 1 0;  % Verde para el eje Y
        1 1 0;  % Amarillo para otra cara
        0 1 1;  % Cian para otra cara
        1 0 1;  % Magenta para otra cara
    ];

    % Calcular la rotación a partir del cuaternión
    R = quat2rotm(quaternion); % Convertir cuaternión a matriz de rotación
    transformedVertices = (R * vertices')'; % Rotar los vértices
    transformedVertices = transformedVertices + [0, 0, 5]; % Trasladar el bloque en z

    % Dibujar el cuboide con colores en las caras
    hold on;
    for i = 1:size(faces, 1)
        fill3(transformedVertices(faces(i,:), 1)', transformedVertices(faces(i,:), 2)', transformedVertices(faces(i,:), 3)', ...
              faceColors(i, :), 'FaceAlpha', 0.8);
    end
    axis equal;
    grid on;
    xlim([-5 5]);
    ylim([-5 5]);
    zlim([0 10]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(3);
    hold off;
end

function R = quat2rotm(q)
    % Convertir cuaternión a matriz de rotación
    q = q(:)'; % Asegurarse de que sea fila
    R = [
        1 - 2*q(3)^2 - 2*q(4)^2, 2*q(2)*q(3) - 2*q(4)*q(1), 2*q(2)*q(4) + 2*q(3)*q(1);
        2*q(2)*q(3) + 2*q(4)*q(1), 1 - 2*q(2)^2 - 2*q(4)^2, 2*q(3)*q(4) - 2*q(2)*q(1);
        2*q(2)*q(4) - 2*q(3)*q(1), 2*q(3)*q(4) + 2*q(2)*q(1), 1 - 2*q(2)^2 - 2*q(3)^2
    ];
end

