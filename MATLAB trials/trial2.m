vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
%%
if (clientID<0)
    disp('Failed connecting to remote API server');
else
    tic;
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    
    [res,MotorHandle_Left]=vrep.simxGetObjectHandle(clientID,'LeftMotor',vrep.simx_opmode_blocking);
    [res,MotorHandle_Right]=vrep.simxGetObjectHandle(clientID,'RightMotor',vrep.simx_opmode_blocking);
    [res,tipHandle]=vrep.simxGetObjectHandle(clientID,'tip',vrep.simx_opmode_blocking);
    [res,tarHandle]=vrep.simxGetObjectHandle(clientID,'tar',vrep.simx_opmode_blocking);
    
    vrep.simxGetObjectPosition(clientID,tarHandle,tipHandle,vrep.simx_opmode_streaming);    
    vrep.simxGetObjectOrientation(clientID,tarHandle,tipHandle,vrep.simx_opmode_streaming);
    state=0;
    
    while toc<30
        [res,position]=vrep.simxGetObjectPosition(clientID,tarHandle,tipHandle,vrep.simx_opmode_streaming);
        [res,orientation]=vrep.simxGetObjectOrientation(clientID,tarHandle,tipHandle,vrep.simx_opmode_streaming);
        dist=sqrt(position(1)^2+position(2)^2);
        theta=atan2(position(2),position(1));
        adtheta=orientation(3);
        
        if state==0
            Left_vel=(dist-theta)*5;
            Right_vel=(dist+theta)*5;
            if abs(position(1))<0.03 && abs(position(2))<0.03
                state=1;
            end
        elseif state==1
            Left_vel=-adtheta*1.5;
            Right_vel=adtheta*1.5;
            if abs(adtheta)<pi/180
               state=2; 
            end
        elseif state==2
            Left_vel=0;
            Right_vel=0;
            if abs(position(1))>0.04 || abs(position(2))>0.04 || abs(adtheta)>2*pi/180
                state=0;
            end
        end
        vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Left,Left_vel,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Right,Right_vel,vrep.simx_opmode_oneshot);
        pause(0.01);
    end

end
%%
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(clientID);
vrep.delete();