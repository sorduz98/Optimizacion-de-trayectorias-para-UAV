function traj = optEnergia(delta, wayi, wayf)
    restaPos=wayi-wayf;
    deltaX=min([wayi(1); wayf(1)]);
    wayi(1)=wayi(1)-deltaX;
    wayf(1)=wayf(1)-deltaX;
    distanciaTotal=sqrt(restaPos(1)^2+restaPos(2)^2+restaPos(3)^2);
    t_final=distanciaTotal*(10/9);
    timeDoc(t_final,delta);
    ecuacionesDoc(wayi, wayf);
    [z, output]=algoritmo_IPOPT();
    traj.output=output;
    traj.costo = z.of(end);
    traj.of = z.of;
    traj.time = z.time;
    traj.x = z.x+deltaX;
    traj.y = z.y;
    traj.z = z.z;
    traj.xp = z.xp;
    traj.yp = z.yp;
    traj.zp = z.zp;
    traj.xpp = z.xpp;
    traj.ypp = z.ypp;
    traj.zpp = z.zpp;
    traj.pitch = z.pitch;
    traj.roll = z.roll;
    traj.yaw = z.yaw;
    traj.pitchp = z.pitchp;
    traj.rollp = z.rollp;
    traj.yawp = z.yawp;
    traj.pitchpp = z.pitchpp;
    traj.rollpp = z.rollpp;
    traj.yawpp = z.yawpp;
    traj.w1 = z.w1;
    traj.w2 = z.w2;
    traj.w3 = z.w3;
    traj.w4 = z.w4;
    traj.t1 = z.t1;
    traj.t2 = z.t2;
    traj.t3 = z.t3;
    traj.t4 = z.t4;
end