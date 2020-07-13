%---------MAIN GENERAL DEL PROYECTO-----------------------
%Establece los waypoints y la presicion de tiempo del algoritmo
%de optimización de energia
delta = 0.1;

waypoints=[0 0 5;      %1
           0 12 5;     %2
           10 12 5;    %3
           10 0 5;     %4
           20 0 5;     %5
           20 12 5;    %6 
           30 12 5;    %7
           30 0 5;     %8
           40 0 5;     %9
           40 12 5];   %10
%Genera las relaciones entre waypoints
costos = [1 0 2; 1 0 3; 1 0 4;
          2 0 1; 2 0 3; 2 0 4;
          3 0 1; 3 0 2; 3 0 4; 3 0 5; 3 0 6;
          4 0 1; 4 0 2; 4 0 3; 4 0 5; 4 0 6;
          5 0 3; 5 0 4; 5 0 6; 5 0 7; 5 0 8;
          6 0 3; 6 0 4; 6 0 5; 6 0 7; 6 0 8;
          7 0 5; 7 0 6; 7 0 8; 7 0 9; 7 0 10;
          8 0 5; 8 0 6; 8 0 7; 8 0 9; 8 0 10;
          9 0 7; 9 0 8; 9 0 10;
          10 0 7; 10 0 8; 10 0 9];
%Calcula los costos de los tramos entre los waypoints
[costos, trajs, tiempo] = costos_trajMapa(delta, waypoints, costos);
%Halla la ruta optima
[OptTraj, cost_rute] = hamiltonian(waypoints, costos);
plotMaptraj(waypoints, OptTraj)
%Genera la trayectoria optima
Vk=1;      %Velocidad maxima
pt=30;     %Porcentaje de aceleración
tm=5/30;   %Tiempo de muestreo

traj_total=traj_planner(OptTraj, Vk, pt, tm);

X=traj_total(:,1);
Y=traj_total(:,2);
Z=traj_total(:,3);
T=traj_total(:,10);

figure
plot3(X,Y,Z,'r*')
xlabel('X (m)','FontSize',26)
ylabel('Y (m)','FontSize',26)
zlabel('Z (m)','FontSize',26)

%Exporta los datos a Simulink
signal_1 = struct('values', X, 'dimensions', 1);
Pos_x = struct('time', T, 'signals', signal_1);
 
signal_2 = struct('values', Y, 'dimensions', 1);
Pos_y = struct('time', T, 'signals', signal_2);
 
signal_3 = struct('values', Z, 'dimensions', 1);
Pos_z = struct('time', T, 'signals', signal_3);


%---------------------------------------------------------
