function [costos, trajs, tiempo] = costos_trajMapa(delta, waypoints, costos)
    tiempo=zeros(size(costos,1),1);
    for i=1:size(costos,1)
        wayi=waypoints(costos(i,1),:)
        wayf=waypoints(costos(i,3),:)
        traj=optEnergia(delta, wayi, wayf);
        trajs(i)=traj;
        costos(i,2)=traj.costo;
        tiempo(i)=traj.time(end);
    end
end