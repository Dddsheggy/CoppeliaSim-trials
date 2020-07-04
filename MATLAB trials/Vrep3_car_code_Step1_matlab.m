vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);%����һ����vrep�����ӣ���û�п�ʼ����
if (clientID<0)
    %���ܵ�ԭ��1.vrep���û�д򿪣�2.�˿ں����ò���(����vrepû�д���صĶ˿ں�)
    disp('Failed connecting to remote API server');    
else
    %�������vrep�ɹ���ִ������Ŀ��Ƴ���
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);%��仰�������������棬�൱������vrep���������水ť
    %��ʼ������
    [res,MotorHandle_Left] = vrep.simxGetObjectHandle(clientID,'LeftMotor',vrep.simx_opmode_blocking);%��ȡ�������Ҫ��ʲô������в�������Ҫ�Ȼ�ȡ���
    [res,MotorHandle_Right] = vrep.simxGetObjectHandle(clientID,'RightMotor',vrep.simx_opmode_blocking);
    %���������ѣ������res�����Ǻ����ô���ֱ�ӱ�ʾ�����������Ƿ�ɹ�����������������ˣ��鿴һ��res�ǲ��Ƿ���vrep.simx_return_ok 
    tic;%�򿪶�ʱ������������15s���Զ�ֹͣ�����԰����Լ���������и���
    
    Left_vel = 20;%���������ؽڵ�ת��
	Right_vel = -20;
	
    while toc<15
        vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Left,Left_vel,vrep.simx_opmode_oneshot  );%��matlab�Ŀ���ָ��䵽vrep�в���Ч
        vrep.simxSetJointTargetVelocity(clientID,MotorHandle_Right,Right_vel,vrep.simx_opmode_oneshot  );
        pause(0.1);
    end
end
%������β����
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
vrep.simxFinish(clientID);
vrep.delete(); % call the destructor!
