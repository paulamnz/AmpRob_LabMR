% AMPLIACION DE ROBOTICA
% PRACTICA 4: Navegacion local con campos potenciales
% Evitar obstaculos

clc
clearvars
close all
%% Carga del mapa de ocupacion

map_img=imread('mapa1_150.png');
map_neg=imcomplement(map_img);
map_bin=imbinarize(map_neg);
mapa=binaryOccupancyMap(map_bin);
show(mapa);

% Marcar los puntos de inicio y destino
hold on;
title('Señala los puntos inicial y final de la trayectoria del robot');
origen=ginput(1);
plot(origen(1), origen(2), 'go','MarkerFaceColor','green');  % Dibujamos el origen
destino=ginput(1);
plot(destino(1), destino(2), 'ro','MarkerFaceColor','red');  % Dibujamos el destino

% Configuracion del sensor (laser de barrido)
max_rango=10;
angulos=-pi/2:(pi/180):pi/2; % resolucion angular barrido laser

% Caracteristicas del vehiculo y parametros del metodo
v=0.4;            % Velocidad del robot
D=1.5;           % Rango del efecto del campo de repulsión de los obstáculos
alfa=1;           % Coeficiente de la componente de atracción
beta=100;      % Coeficiente de la componente de repulsión

%% Inicialización

robot=[origen 0];     % El robot empieza en la posición de origen (orientacion cero)
path = [];                 % Se almacena el camino recorrido
path = [path; robot]; % Se añade al camino la posicion actual del robot
iteracion=0;              % Se controla el nº de iteraciones por si se entra en un minimo local

%% Calculo de la trayectoria

while norm(destino-robot(1:2)) > v && iteracion<1000    % Hasta menos de una iteración de la meta (10 cm)
   
    %Vamos a calcular solo las fuerzas de atraccion y repulsion en la
    %ubicacion del robot

    %Fres = Fatr(q) + suma(Frep(q))

    %Las fuerzas de atracción serán
    %alfa*(qdestino - q)

    Fatr = alfa*(destino - robot(1:2));

    %Las fuerzas de repulsión serán
    sum_Frep = [0, 0];
    %beta* (1/rho_obs - 1/D)*(q-obst)/rho_obs^3 si rho_obs <=D ; si no, 0
    %rho_obs = dist(q,obst)

    %Para sacar los obstáculos barriremos el láser
    array_obst = SimulaLidar(robot,mapa,angulos,max_rango);

    for a = 1:1:size(array_obst)
        obst = array_obst(a,:);

        %nos saltamos valores NaN

        if any(isnan(obst))
            continue
        end

        rho_obs = norm(robot(1:2) - obst);
        if rho_obs < D
            Frep = beta* ((1/rho_obs) - (1/D))*(robot(1:2)-obst)/(rho_obs^3);
        else 
            Frep = [0,0];
        end
        sum_Frep = sum_Frep + Frep;
    end

    % La fuerza resultante es
    Fres = Fatr + sum_Frep;

    %Descomentar para visualizar las fuerzas
    %quiver(robot(1), robot(2), Fatr(1), Fatr(2), 0.5, 'g', 'LineWidth', 2);      % green = attraction
    %quiver(robot(1), robot(2), sum_Frep(1), sum_Frep(2), 0.5, 'r', 'LineWidth', 2); % red = repulsion
    %quiver(robot(1), robot(2), Fres(1), Fres(2), 0.5, 'b', 'LineWidth', 2);      % blue = resultant
    %drawnow


    % Y la orientación es 
    %theta = atan2(Fres(2), Fres(1));
    if norm(Fres) > 0
        direction = Fres / norm(Fres);
    end
    theta = atan2(direction(2), direction(1));

    %robot = [robot(1) + v*cos(theta), robot(2) + v*sin(theta), theta];
    robot = [robot(1) + v*direction(1), robot(2) + v*direction(2), theta];

    path = [path;robot];	% Se añade la nueva posición al camino seguido
    plot(path(:,1),path(:,2),'r');
    drawnow

    iteracion=iteracion+1;
end

if iteracion==1000   % Se ha caído en un mínimo local
    fprintf('No se ha podido llegar al destino.\n')
else
    fprintf('Destino alcanzado.\n')
end

%% funcion para simular el sensor
function [obs]=SimulaLidar(robot, mapa, angulos, max_rango)
    obs=rayIntersection(mapa,robot,angulos, max_rango);
    % plot(obs(:,1),obs(:,2),'*r') % Puntos de interseccion lidar
    % plot(robot(1),robot(2),'ob') % Posicion del robot
    % for i = 1:length(angulos)
    %     plot([robot(1),obs(i,1)],...
    %         [robot(2),obs(i,2)],'-b') % Rayos de interseccion
    % end
    % % plot([robot(1),robot(1)-6*sin(angulos(4))],...
    % %     [robot(2),robot(2)+6*cos(angulos(4))],'-b') % Rayos fuera de
    % %     rango
end