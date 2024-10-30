clc;           % Limpia la ventana de comandos
clear;         % Limpia las variables del espacio de trabajo
close all;     % Cierra todas las figuras abiertas

% Parámetros de la trayectoria deseada (círculo)
cx = 2; 
cy = 8; 
r = 2;               % Centro y radio del círculo
Px = cx + r * cosd(0:6:360);         % Coordenadas x de la trayectoria
Py = cy + r * sind(0:6:360);         % Coordenadas y de la trayectoria

% Solución de las ecuaciones y simulación del mecanismo
figure(1)  % Crear una nueva figura para la animación

% Longitudes conocidas de las barras y distancia entre puntos fijos
L2 = 3.5; L3 = 7; L4 = 7; L5 = 3.4; 
O2O5 = 4;  % Distancia entre los puntos O2 y O5 (puntos fijos en la base)

% Valores iniciales de los ángulos (estimación inicial)
x0 = [100 40 100 40];

% Inicializar los vectores para almacenar los ángulos calculados
th2_values = zeros(1, numel(Px));
th3_values = zeros(1, numel(Px));
th4_values = zeros(1, numel(Px));
th5_values = zeros(1, numel(Px));

% Iterar a través de cada punto de la trayectoria
for i = 1:numel(Px)
    % Coordenadas actuales del punto objetivo (Bx, By)
    Bx = Px(i); By = Py(i);
    c = [O2O5 L2 L3 L4 L5 Bx By];  % Vector de parámetros constantes
    
    % Resolver las ecuaciones para obtener los ángulos usando fminsearch
    objFunc = @(x) norm(SoluCineInv(x, c)); % Definir función objetivo para minimización
    x = fminsearch(objFunc, x0);  % Usar fminsearch para minimizar la función objetivo
    
    % Guardar los valores calculados para θ2, θ3, θ4 y θ5 en los vectores
    th2_values(i) = x(1);
    th3_values(i) = x(2);
    th4_values(i) = x(3);
    th5_values(i) = x(4);

    % Calcular la posición de los puntos clave del mecanismo
    A = L2 * [cosd(x(1)) sind(x(1))];       % Punto A (extremo de la barra 2)
    B = A + L3 * [cosd(x(2)) sind(x(2))];   % Punto B (extremo de la barra 3)
    C = B - L4 * [cosd(x(3)) sind(x(3))];   % Punto C (extremo de la barra 4)
    O2 = [0 0];                             % Punto O2 (origen)
    O5 = [O2O5 0];                          % Punto O5 (fijo en la base)

    % Graficar la trayectoria y los puntos del mecanismo
    figure(1)
    plot(Px, Py, '-g'), hold on        % Trazo de la trayectoria circular
    plot(A(1), A(2), 'or')             % Punto A (en rojo)
    plot(B(1), B(2), 'or')             % Punto B (en rojo)
    plot(C(1), C(2), 'or')             % Punto C (en rojo)
    plot(O2(1), O2(2), 'or')           % Punto O2 (en rojo)
    plot(O5(1), O5(2), 'or')           % Punto O5 (en rojo)
    
    % Dibujar las barras del mecanismo
    plot([O2(1) A(1)], [O2(2) A(2)], 'k')  % Barra 2
    plot([B(1) A(1)], [B(2) A(2)], 'k')    % Barra 3
    plot([B(1) C(1)], [B(2) C(2)], 'k')    % Barra 4
    plot([O5(1) C(1)], [O5(2) C(2)], 'k')  % Barra 5
    
    hold off
    xlim([-5 10]), ylim([-5 15])  % Límites de los ejes para mejor visualización
    drawnow                        % Actualizar la figura en cada iteración
end

% Mostrar los valores calculados (opcional, para ver los resultados)
disp('Valores de th2:');
disp(th2_values);
disp('Valores de th3:');
disp(th3_values);
disp('Valores de th4:');
disp(th4_values);
disp('Valores de th5:');
disp(th5_values);

%% Función de Solución de Cinemática Inversa
function y = SoluCineInv(x, c)
    % Variables incógnitas (ángulos)
    th2 = x(1); th3 = x(2); th4 = x(3); th5 = x(4);
    
    % Constantes conocidas (longitudes y posición objetivo)
    o2o5 = c(1); L2 = c(2); L3 = c(3); L4 = c(4); L5 = c(5);
    Bx = c(6); By = c(7);

    % Ecuaciones del sistema para resolver los ángulos
    y(1) = L2 * cosd(th2) + L3 * cosd(th3) - L4 * cosd(th4) - L5 * cosd(th5) - o2o5;
    y(2) = L2 * sind(th2) + L3 * sind(th3) - L4 * sind(th4) - L5 * sind(th5);
    y(3) = L2 * cosd(th2) + L3 * cosd(th3) - Bx;
    y(4) = L2 * sind(th2) + L3 * sind(th3) - By;
end
