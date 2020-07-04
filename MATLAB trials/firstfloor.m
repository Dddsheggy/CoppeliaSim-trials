clc;
close all;

vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
%%
if (clientID<0)
    disp('Failed connecting to remote API server');
else
    tic;
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    
    [res,fl]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_blocking);
    [res,rl]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_blocking);
    [res,rr]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_blocking);
    [res,fr]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_blocking);
    [res,tipHandle]=vrep.simxGetObjectHandle(clientID,'tip',vrep.simx_opmode_blocking);
    [res,tarHandle]=vrep.simxGetObjectHandle(clientID,'tar',vrep.simx_opmode_blocking);
    
    vrep.simxGetObjectPosition(clientID,tarHandle,tipHandle,vrep.simx_opmode_streaming);    
    vrep.simxGetObjectOrientation(clientID,tarHandle,tipHandle,vrep.simx_opmode_streaming);
    state=0;
    flag1=0;
    flag2=0;
    
    while toc<20
        [res,position]=vrep.simxGetObjectPosition(clientID,tarHandle,tipHandle,vrep.simx_opmode_streaming);
        [res,orientation]=vrep.simxGetObjectOrientation(clientID,tarHandle,tipHandle,vrep.simx_opmode_streaming);
        dist=sqrt(position(1)^2+position(2)^2);
        theta=atan2(position(2),position(1));
        adtheta=orientation(3);
        if flag2==1 && abs(theta)<0.06
            flag1=1;
        end
        
        if state==0   
            if flag1
                All_vel=dist;
                Left_vel=dist+theta*20+adtheta*3;
                Right_vel=dist-theta*20-adtheta*3;
%                 All_vel=5*dist;
%                 Left_vel=5*dist;
%                 Right_vel=5*dist;
            else
                All_vel=dist;
                Left_vel=dist+theta*20+adtheta*3;
                Right_vel=dist-theta*20-adtheta*3;
            end           

            if dist<0.08
                state=1;
            end
        elseif state==1
            Left_vel=adtheta*5;
            Right_vel=-adtheta*5;
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
        
        vrep.simxSetJointTargetVelocity(clientID,fl,Left_vel,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID,rl,All_vel,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID,rr,All_vel,vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID,fr,Right_vel,vrep.simx_opmode_oneshot);
        fprintf([num2str(toc), 's', '  ', 'state: ', num2str(state),' dist: ', num2str(dist), ' theta: ',num2str(theta), '  leftvel: ', num2str(Left_vel), '  rightvel: ', num2str(Right_vel), '\n']);
        pause(0.01);
        if toc>1.5
            flag2=1;
        end
    end

end
%%
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(clientID);
vrep.delete();