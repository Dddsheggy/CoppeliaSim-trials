vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

%%
if (clientID<0)
    disp('Failed connecting to remote API server');
else
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    
    [res,MotorHandle_Left]=vrep.simxGetObjectHandle(clientID,'LeftMotor',vrep.simx_opmode_blocking);
    [res,MotorHandle_Right]=vrep.simxGetObjectHandle(clientID,'RightMotor',vrep.simx_opmode_blocking);
    tic;
    Left_vel=10;
    Right_vel=-10;
    
    while toc<10
        vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Left,Left_vel,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Right,Right_vel,vrep.simx_opmode_oneshot);
        pause(0.1);
    end
end
%%
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(clientID);
vrep.delete();