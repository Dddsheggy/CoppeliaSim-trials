vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
if (clientID<0)
    disp('Failed connecting to remote API server');    
else
    tic;
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    %初始化
    %获得被控对象的句柄
    [res,MotorHandle_Left] = vrep.simxGetObjectHandle(clientID,'LeftMotor',vrep.simx_opmode_blocking);
    [res,MotorHandle_Right] = vrep.simxGetObjectHandle(clientID,'RightMotor',vrep.simx_opmode_blocking);
    [res,tip_handle] = vrep.simxGetObjectHandle(clientID,'tip',vrep.simx_opmode_blocking);
    [res,tar_handle] = vrep.simxGetObjectHandle(clientID,'tar',vrep.simx_opmode_blocking);
    %初始化一些变量
    state = 0;%采用状态机实现，用于标识当前状态。0表示完成，1表示趋于目标位置，2表示对齐方位
    
    %执行第一次操作（第一次使用时采用simx_opmode_streaming，然后后面的采用simx_opmode_buffer）
    vrep.simxGetObjectPosition(clientID,tar_handle,tip_handle,vrep.simx_opmode_streaming);
    vrep.simxGetObjectOrientation(clientID,tar_handle,tip_handle,vrep.simx_opmode_streaming);
    %这种方式是两个dummy的相对坐标，如果是求相对于时间坐标系的坐标的话，把tip_handle改成-1。
    
    
    while toc<30
        [res,Dpos] = vrep.simxGetObjectPosition(clientID,tar_handle,tip_handle,vrep.simx_opmode_buffer );
        [res,Dangle] = vrep.simxGetObjectOrientation(clientID,tar_handle,tip_handle,vrep.simx_opmode_buffer );
        d= sqrt( Dpos(1)^2 + Dpos(2)^2 );
        theta = atan2(Dpos(2),Dpos(1));
        Vtheta = Dangle(3)*5;
        
        if state == 0 
            %调整小车车速和朝向，达到目标位置，转入状态1进入调整小车朝向
            Left_vel = d*5 - theta * 5;
            Right_vel = d*5 + theta * 5;
            if abs(Dpos(1))<0.03 && abs(Dpos(2))<0.03
                state = 1;%转移到状态1
            end
        elseif state ==1
            %调整小车角度，将小车完全停入目标区域，达到目标位置后转入状态2停止运动。
            Left_vel = -Vtheta * 0.25;
            Right_vel =Vtheta * 0.25;
            if abs(Dangle(3))<1*pi/180
                state = 2;
            end
        elseif state ==2
            %达到目标位置，使小车停止。同时检测目标位置是否有改动，如果有的话，跳转到状态0
            Left_vel=0;
            Right_vel = 0;
            if abs(Dpos(1))>0.04 || abs(Dpos(2))>0.04 || abs(Dangle(3))>2*pi/180
                state =0;
            end
        end
        vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Left,Left_vel,vrep.simx_opmode_oneshot  );
        vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Right,Right_vel,vrep.simx_opmode_oneshot  );
        pause(0.01);
        
        
    end
end
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(clientID);
vrep.delete(); % call the destructor!
