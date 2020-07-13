%{
Planeacion de la trayectoria total a seguir por el UAV
a partir de los waypoints con trayectoria optima.
Esta compuesta por dos funciones, traj_plan y traj_planner.

Traj_plan se encarga de generar la trayectoria entre dos 
waypoints.

Traj_planner se encarga de generar la trayectoria de 
todo el recorrido, reutilizando la funcion traj_planner
para generar la trayectoria de cada tramo entre los 
waypoints para obtener la trayectoria completa de la 
mision.

Requiere:
    Vk = velocidad maxima del UAV
    pt = Porcentaje de aceleracion
    tm = tiempo de muestro
    waypoints = Matriz que contiene los waypoints a recorrer
                de forma ordenada.
                La matriz debe tener la siguiente forma:
                waypoints = [x1 y1 z1;
                             x2 y2 z2;
                             ... ;
                             xn yn zn]

Retorna:
    traj = Matriz que comprende la trayecoria total de la
           mision. Contiene todos los valores de posicion, 
           velocidad, aceleracion y tiempo.

           traj= [X Y Z Vx Vy Vz Ax Ay Az t;
                  X Y Z Vx Vy Vz Ax Ay Az t;
                            ...
                  X Y Z Vx Vy Vz Ax Ay Az t]
    n = numero de puntos
    T = tiempo total
%}
function traj_total=traj_planner(waypoints, Vk, pt, tm)
    sz=size(waypoints);
    n_way=sz(1,1);
    T_actual=0;
    T=0;
    n=0;
    traj_total=zeros(1,10);
    for i= 1:n_way-1
        posiciones=[waypoints(i,:) ; waypoints(i+1,:)];
        [traj,T,n]=traj_plan(posiciones, Vk, pt, tm);
        traj(:,10)=traj(:,10)+T_actual;
        traj_total=[traj_total ; traj];
        T_actual=T_actual+T;
    end
    traj_total=traj_total(2:end,:);
end

function [traj,T,n] = traj_plan(posiciones, Vk, pt, tm)
    pt=pt/100;
    posiciones=posiciones.';
    P0=posiciones(1:3);
    PF=posiciones(4:6);
    restaPos=PF-P0;
    distanciaTotal=sqrt(restaPos(1)^2+restaPos(2)^2+restaPos(3)^2);
    T=distanciaTotal/(Vk*(1-pt));
    n=round(T/tm);
    k=round(pt*n);
    kc=n-2*k;
    Tau=T*pt;
    a=Vk/Tau;


    X_pt=(1/(2*(distanciaTotal)))*a*Tau^2*restaPos(1)+P0(1);
    Y_pt=(1/(2*(distanciaTotal)))*a*Tau^2*restaPos(2)+P0(2);
    Z_pt=(1/(2*(distanciaTotal)))*a*Tau^2*restaPos(3)+P0(3);

    P_t=[X_pt Y_pt Z_pt];

    X_pT_t=((Vk*(T-2*Tau))/distanciaTotal)*restaPos(1)+X_pt;
    Y_pT_t=((Vk*(T-2*Tau))/distanciaTotal)*restaPos(2)+Y_pt;
    Z_pT_t=((Vk*(T-2*Tau))/distanciaTotal)*restaPos(3)+Z_pt;

    P_Tt=[X_pT_t Y_pT_t Z_pT_t];

%--------------------------TRAMO ACELERACION-------------------------------

    disT1=sqrt((P_t(1)-P0(1))^2+(P_t(2)-P0(2))^2+(P_t(3)-P0(3))^2);
    Tseg1=2*disT1/Vk;
    a=Vk/Tseg1;

    tk_seg1=[0:Tseg1/k:Tseg1];

    X_seg1=(1/(2*disT1))*a*tk_seg1.^2*(P_t(1)-P0(1))+P0(1);
    Y_seg1=(1/(2*disT1))*a*tk_seg1.^2*(P_t(2)-P0(2))+P0(2);
    Z_seg1=(1/(2*disT1))*a*tk_seg1.^2*(P_t(3)-P0(3))+P0(3);

    Vx_seg1=(1/(2*disT1))*a*tk_seg1*2*(P_t(1)-P0(1));
    Vy_seg1=(1/(2*disT1))*a*tk_seg1*2*(P_t(2)-P0(2));
    Vz_seg1=(1/(2*disT1))*a*tk_seg1*2*(P_t(3)-P0(3));

    Ax_seg1=(1/(2*disT1))*a*2*(P_t(1)-P0(1))*ones(size(tk_seg1));
    Ay_seg1=(1/(2*disT1))*a*2*(P_t(2)-P0(2))*ones(size(tk_seg1));
    Az_seg1=(1/(2*disT1))*a*2*(P_t(3)-P0(3))*ones(size(tk_seg1));

%---------------------TRAMO VELOCIDAD COSNTANTE----------------------------

    disT2=sqrt((P_Tt(1)-P_t(1))^2+(P_Tt(2)-P_t(2))^2+(P_Tt(3)-P_t(3))^2);
    Tseg2=disT2/Vk;

    tk_seg2=[0:Tseg2/kc:Tseg2];

    X_seg2=((Vk*tk_seg2)/disT2)*(P_Tt(1)-P_t(1))+P_t(1);
    Y_seg2=((Vk*tk_seg2)/disT2)*(P_Tt(2)-P_t(2))+P_t(2);
    Z_seg2=((Vk*tk_seg2)/disT2)*(P_Tt(3)-P_t(3))+P_t(3);

    Vx_seg2=(Vk/disT2)*(P_Tt(1)-P_t(1))*ones(size(tk_seg2));
    Vy_seg2=(Vk/disT2)*(P_Tt(2)-P_t(2))*ones(size(tk_seg2));
    Vz_seg2=(Vk/disT2)*(P_Tt(3)-P_t(3))*ones(size(tk_seg2));

    Ax_seg2=zeros(size(tk_seg2));
    Ay_seg2=zeros(size(tk_seg2));
    Az_seg2=zeros(size(tk_seg2));

%--------------------TRAMO ACELERACION NEGATIVA----------------------------

    disT3=sqrt((PF(1)-P_Tt(1))^2+(PF(2)-P_Tt(2))^2+(PF(3)-P_Tt(3))^2);
    Tseg3=2*disT3/Vk;
    a=-Vk/Tseg3;

    tk_seg3=[0:Tseg3/k:Tseg3];

    X_seg3=(1/disT3)*(1/2*a*tk_seg3.^2+Vk*tk_seg3)*(PF(1)-P_Tt(1))+P_Tt(1);
    Y_seg3=(1/disT3)*(1/2*a*tk_seg3.^2+Vk*tk_seg3)*(PF(2)-P_Tt(2))+P_Tt(2);
    Z_seg3=(1/disT3)*(1/2*a*tk_seg3.^2+Vk*tk_seg3)*(PF(3)-P_Tt(3))+P_Tt(3);

    Vx_seg3=(1/disT3)*(a*tk_seg3+Vk)*(PF(1)-P_Tt(1));
    Vy_seg3=(1/disT3)*(a*tk_seg3+Vk)*(PF(2)-P_Tt(2));
    Vz_seg3=(1/disT3)*(a*tk_seg3+Vk)*(PF(3)-P_Tt(3));

    Ax_seg3=(1/disT3)*(a)*(PF(1)-P_Tt(1))*ones(size(tk_seg3));
    Ay_seg3=(1/disT3)*(a)*(PF(2)-P_Tt(2))*ones(size(tk_seg3));
    Az_seg3=(1/disT3)*(a)*(PF(3)-P_Tt(3))*ones(size(tk_seg3));

%------------------------GENERAR TRAYECTORIA-------------------------------

    X=[X_seg1 X_seg2(2:(size(X_seg2.')-1)) X_seg3].';
    Y=[Y_seg1 Y_seg2(2:(size(Y_seg2.')-1)) Y_seg3].';
    Z=[Z_seg1 Z_seg2(2:(size(Z_seg2.')-1)) Z_seg3].';

    Vx=[Vx_seg1 Vx_seg2(2:(size(Vx_seg2.')-1)) Vx_seg3].';
    Vy=[Vy_seg1 Vy_seg2(2:(size(Vy_seg2.')-1)) Vy_seg3].';
    Vz=[Vz_seg1 Vz_seg2(2:(size(Vz_seg2.')-1)) Vz_seg3].';

    Ax=[Ax_seg1 Ax_seg2(2:(size(Ax_seg2.')-1)) Ax_seg3].';
    Ay=[Ay_seg1 Ay_seg2(2:(size(Ay_seg2.')-1)) Ay_seg3].';
    Az=[Az_seg1 Az_seg2(2:(size(Az_seg2.')-1)) Az_seg3].';

    t=[0:T/(2*k+kc):T].';
    V=sqrt(Vx.^2+Vy.^2+Vz.^2);
    traj= [X Y Z Vx Vy Vz Ax Ay Az t];
end


