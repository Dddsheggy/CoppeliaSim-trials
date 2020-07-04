vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);%启动一个与vrep的连接，并没有开始仿真
if (clientID<0)
    %可能的原因：1.vrep软件没有打开；2.端口号设置不对(或者vrep没有打开相关的端口号)
    disp('Failed connecting to remote API server');    
else
    %如果连接vrep成功，执行下面的控制程序
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);%这句话是用于启动仿真，相当于你点击vrep的启动仿真按钮
    %初始化工作
    [res,MotorHandle_Left] = vrep.simxGetObjectHandle(clientID,'LeftMotor',vrep.simx_opmode_blocking);%获取句柄，你要对什么对象进行操作，需要先获取句柄
    [res,MotorHandle_Right] = vrep.simxGetObjectHandle(clientID,'RightMotor',vrep.simx_opmode_blocking);
    %！！！提醒，这里的res并不是毫无用处，直接表示了你语句操作是否成功，如果你代码出问题了，查看一下res是不是返回vrep.simx_return_ok 
    tic;%打开定时器，程序运行15s后自动停止，可以按照自己的需求进行更改
    
    Left_vel = 20;%设置两个关节的转速
	Right_vel = -20;
	
    while toc<15
        vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Left,Left_vel,vrep.simx_opmode_oneshot  );%将matlab的控制指令传输到vrep中并生效
        vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Right,Right_vel,vrep.simx_opmode_oneshot  );
        pause(0.1);
    end
end
%处理收尾工作
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(clientID);
vrep.delete(); % call the destructor!
