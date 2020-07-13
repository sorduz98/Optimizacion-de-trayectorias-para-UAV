%%Posición inicial UAV
x=(X-20)/2.5;
y=(Y-6)/2.5;
z=Z-1.5;
position=[x,y,z];
%load('Trayectoria optima.mat');
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if clientID~=-1
    message='Connected to remote API server';
    disp(message);
    
    %%How to get the HANDLES 
    [returnCode,target]=vrep.simxGetObjectHandle(clientID,'Quadricopter_target',vrep.simx_opmode_blocking);
       
    %%How to modify the handles
    [returnCode]=vrep.simxSetObjectPosition(clientID,target,-1,position,vrep.simx_opmode_oneshot);
    
    for i=1:size(x)
        position=[x(i),y(i),z(i)];
        [returnCode]=vrep.simxSetObjectPosition(clientID,target,-1,position,vrep.simx_opmode_oneshot);
        pause(0.15);
    end
end