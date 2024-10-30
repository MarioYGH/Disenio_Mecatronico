clc;           % Borra la consola
clear;         % Libera las variables en el espacio de trabajo
close all;     % Cierra todas las figuras abiertas

% Definición de la posición del centro y el radio del hexágono
cx = 2; 
cy = 8; 
r = 2;                   % Radio del hexágono
num_vertices = 6;        % Número de vértices del hexágono
num_intermediate = 10;   % Puntos intermedios en cada lado

% Cálculo de los ángulos para los vértices del hexágono
theta_hex = linspace(0, 360, num_vertices + 1);

% Coordenadas (x, y) de los vértices principales del hexágono
Px_vertices = cx + r * cosd(theta_hex);
Py_vertices = cy + r * sind(theta_hex);

% Generación de puntos intermedios a lo largo de los lados del hexágono
Px = [];
Py = [];
for i = 1:num_vertices
    % Interpolación entre los vértices para generar puntos intermedios
    x_interp = linspace(Px_vertices(i), Px_vertices(i + 1), num_intermediate + 1);
    y_interp = linspace(Py_vertices(i), Py_vertices(i + 1), num_intermediate + 1);
    
    % Almacena los puntos intermedios, evitando duplicar el último punto
    Px = [Px, x_interp(1:end-1)];
    Py = [Py, y_interp(1:end-1)];
end
% Cierra el hexágono conectando el último punto con el primero
Px = [Px, Px(1)];
Py = [Py, Py(1)];

% Inicialización de la simulación del mecanismo de 5 barras
figure(1)  % Crear una ventana de figura

% Configuración de las longitudes de las barras y la distancia entre los puntos fijos
L2 = 4; L3 = 7; L4 = 6; L5 = 4; 
O2O5 = 5;  % Distancia entre los puntos fijos O2 y O5

% Valores iniciales estimados para los ángulos
x0 = [100 40 100 40];

% Inicialización de vectores para almacenar los ángulos resultantes
th2_values = zeros(1, numel(Px));
th3_values = zeros(1, numel(Px));
th4_values = zeros(1, numel(Px));
th5_values = zeros(1, numel(Px));

% Cálculo y simulación de la cinemática inversa en cada punto de la trayectoria
for i = 1:numel(Px)
    % Coordenadas del objetivo para la posición deseada del mecanismo
    Bx = Px(i); By = Py(i);
    c = [O2O5 L2 L3 L4 L5 Bx By];  % Vector de constantes para la cinemática
    
    % Resolución de la cinemática inversa usando optimización para encontrar los ángulos
    objFunc = @(x) norm(SoluCineInv(x, c)); % Definición de la función de minimización
    x = fminsearch(objFunc, x0);            % Optimización con fminsearch
    
    % Almacenamiento de los ángulos calculados en sus respectivos vectores
    th2_values(i) = x(1);
    th3_values(i) = x(2);
    th4_values(i) = x(3);
    th5_values(i) = x(4);

    % Cálculo de la posición de los puntos clave del mecanismo
    A = L2 * [cosd(x(1)) sind(x(1))];        % Coordenadas del punto A
    B = A + L3 * [cosd(x(2)) sind(x(2))];    % Coordenadas del punto B
    C = B - L4 * [cosd(x(3)) sind(x(3))];    % Coordenadas del punto C
    O2 = [0 0];                              % Origen (O2)
    O5 = [O2O5 0];                           % Punto fijo O5

    % Dibujo de la trayectoria y los puntos clave del mecanismo
    figure(1)
    plot(Px, Py, '-g'), hold on               % Traza la trayectoria deseada en verde
    plot(A(1), A(2), 'or')                    % Dibuja el punto A
    plot(B(1), B(2), 'or')                    % Dibuja el punto B
    plot(C(1), C(2), 'or')                    % Dibuja el punto C
    plot(O2(1), O2(2), 'or')                  % Dibuja el punto fijo O2
    plot(O5(1), O5(2), 'or')                  % Dibuja el punto fijo O5
    
    % Dibuja las barras del mecanismo que conectan los puntos
    plot([O2(1) A(1)], [O2(2) A(2)], 'k')     % Barra de O2 a A
    plot([B(1) A(1)], [B(2) A(2)], 'k')       % Barra de A a B
    plot([B(1) C(1)], [B(2) C(2)], 'k')       % Barra de B a C
    plot([O5(1) C(1)], [O5(2) C(2)], 'k')     % Barra de O5 a C
    
    hold off
    xlim([-5 10]), ylim([-5 15])              % Establece los límites de la gráfica
    drawnow                                   % Actualiza la figura en cada paso
end

% Mostrar los valores finales de los ángulos (opcional para análisis)
disp('Valores de th2:');
disp(th2_values);
disp('Valores de th3:');
disp(th3_values);
disp('Valores de th4:');
disp(th4_values);
disp('Valores de th5:');
disp(th5_values);

%% Función de resolución de la cinemática inversa
function y = SoluCineInv(x, c)
    % Variables incógnitas correspondientes a los ángulos
    th2 = x(1); th3 = x(2); th4 = x(3); th5 = x(4);
    
    % Constantes del mecanismo (longitudes y posición objetivo)
    o2o5 = c(1); L2 = c(2); L3 = c(3); L4 = c(4); L5 = c(5);
    Bx = c(6); By = c(7);

    % Ecuaciones de la cinemática inversa para calcular los ángulos
    y(1) = L2 * cosd(th2) + L3 * cosd(th3) - L4 * cosd(th4) - L5 * cosd(th5) - o2o5;
    y(2) = L2 * sind(th2) + L3 * sind(th3) - L4 * sind(th4) - L5 * sind(th5);
    y(3) = L2 * cosd(th2) + L3 * cosd(th3) - Bx;
    y(4) = L2 * sind(th2) + L3 * sind(th3) - By;
end
