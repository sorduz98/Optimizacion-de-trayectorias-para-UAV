%Cristhian Santiago Muñoz
%Juan Sebastian Corredor

%Hoja de parametros

%Parametros UAV

M_t=0.6;        %Masa del drone
P_air=1.26;     %Desnsidad del aire
C_l=1.6;        %Coeficiente Lift
C_d=0.042;      %Coficiente Drag
Jcm_x=7.5e3; %Iniercia en X
Jcm_y=7.5e3; %Iniercia en Y
Jcm_z=14e3;  %Iniercia en Z
g=9.81;         %Gravedad
A=0.013;        %Area del aspa

%Parametros Motor

J=4.1904e-5;
Kt=0.0104e-3;
Kv=96.342;
Dv=0.2e-3;
R=0.2;
%--Coeficientes
a1=-1.72e-5;
a2=1.95e-5;
a3=-6.98e-6;
a4=4.09e-7;
b1=0.014;
b2=-0.0157;
b3=5.656e-3;
b4=-3.908e-4;
c1=-0.8796;
c2=0.3385;
c3=0.2890;
c4=0.1626;

%Parametros Modelo de la bateria

E0=1.2449;
K=0.0029221;
Ab=0.156;
Q=1.5;          %Carga de la bateria en Ah
B=2.3529;
