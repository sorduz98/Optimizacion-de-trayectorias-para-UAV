function [z, output]=algoritmo_IPOPT()
    clc
    server = 'http://127.0.0.1';
    app = 'traj_optima';

    addpath('C:/Program Files/MATLAB/apm_matlab_v0.7.2/apm')
    addpath('D:/Universidad Javeriana/SEMESTRE X/Trabajo de Grado -1934/Algoritmo de optimización/Funciones')

    apm(server,app,'clear all');
    apm_load(server,app,'ecuaciones.apm');
    csv_load(server,app,'tiempo.csv');

    apm_option(server,app,'apm.max_iter',100);
    apm_option(server,app,'apm.nodes',3);
    apm_option(server,app,'apm.rtol',1e-6);
    apm_option(server,app,'apm.otol',1e-6);
    apm_option(server,app,'apm.solver',3);
    apm_option(server,app,'apm.imode',6);
    apm_option(server,app,'apm.mv_type',1);


    costo=1e-5;%1e-5
    %VARIABLES CONTROLADAS
    %Velocidades angulares
    apm_info(server,app,'MV','w1p');
    apm_option(server,app,'w1p.status',1);
    apm_info(server,app,'MV','w2p');
    apm_option(server,app,'w2p.status',1);
    apm_info(server,app,'MV','w3p'); 
    apm_option(server,app,'w3p.status',1);
    apm_info(server,app,'MV','w4p');
    apm_option(server,app,'w4p.status',1);

    %Salida
%     disp('')
%     disp('------------- Initialize ----------------')
%     apm_option(server,app,'apm.coldstart',1);
%     output = apm(server,app,'solve');
%     disp(output)

    disp('')
    disp('-------------- Optimize -----------------')
    apm_option(server,app,'apm.time_shift',0);
    apm_option(server,app,'apm.coldstart',0);
    output = apm(server,app,'solve');
    disp(output)
    output=output(end-238:end-8);
    y = apm_sol(server,app); 
    z = y.x;
end