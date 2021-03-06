ip='172.31.1.147';

arg1=KST.LBR7R800; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
Tef_flange(3,4) = 0.153;
global iiwa;
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object
iiwa.net_establishConnection()

try
rosinit( 'NodeName','matlab_vision_test');
catch e
    rosshutdown
    rosinit( 'NodeName','matlab_vision_test');
end

%% move to init pose


    init_pose = {-211.7920 216.6591 551.3870 -0.7872 0.5240 -3.1415};

    iiwa.movePTPLineEEF(init_pose, 30);{-309 308.5 600 -0.7872 0 -3.1414}



%% get screw positions
try
    pubJoint = rospublisher('/joint_temp', 'sensor_msgs/JointState');
    pubTrigger = rospublisher('/matlab_trigger', 'std_msgs/Empty');

    screwPos = rossubscriber('/vision_positions_pc', 'sensor_msgs/PointCloud');


    jointmsg = rosmessage(pubJoint);
    Joint_names = {'iiwa_joint_1' 'iiwa_joint_2' 'iiwa_joint_3' 'iiwa_joint_4' 'iiwa_joint_5' 'iiwa_joint_6' 'iiwa_joint_7'};
    vel = [0 0 0 0 0 0 0];

    jointmsg.Name = Joint_names;
    jointmsg.Velocity = vel;

    temp_joint_pose = iiwa.getJointsPos();
    temp_JPos = [temp_joint_pose{1} temp_joint_pose{2} temp_joint_pose{3} temp_joint_pose{4}];
    temp_JPos = [temp_JPos temp_joint_pose{5} temp_joint_pose{6} temp_joint_pose{7}];
    
    jointmsg.Position = temp_JPos;
    jointmsg.Header.Stamp = rostime("now");
    
    send(pubJoint, jointmsg);
    empty = rosmessage('std_msgs/Empty');
    send(pubTrigger, empty);
    
    screwResp = receive(screwPos, 10)


    %% go through positions
    
    for i = 1:length(screwResp.Points)
    temp_pos = screwResp.Points(i);

    %% move to screw
        
    pose = {temp_pos.X*1000+2 temp_pos.Y*1000-1.5 400 -0.7872 0 -3.1414}

    iiwa.movePTPLineEEF(pose, 20);
        
    joints_indices=[4]; % joint index start from one
    max_torque=[-5];
    min_torque=[-12];
    
    pose = {temp_pos.X*1000+2 temp_pos.Y*1000-1.5 340 -0.7872 0 -3.1414};
    
    stopped = iiwa.movePTP_ConditionalTorque_LineEEF(pose, 10, joints_indices, max_torque, min_torque)

    jPose = iiwa.getJointsPos();
    iiwa.sendJointsPositionsf(jPose);
    
    %% generate relative pose 
    
    qi = [jPose{1} jPose{2} jPose{3} jPose{4} jPose{5} jPose{6} jPose{7}]';
    dirkin = iiwa.gen_DirectKinematics(qi)

    temp = dirkin;
    temp(3,4) = dirkin(3,4)-0.02;
    
    lambda=0.01;
    n=500;
    qf = iiwa.gen_InverseKinematics( qi, temp,n,lambda )

    jPos = {qf(1) qf(2) qf(3) qf(4) qf(5) qf(6) qf(7)};
   
    %temp1 = iiwa.gen_DirectKinematics( qf(:,1))
%     for i = 1:7
    
    invkin = iiwa.gen_DirectKinematics(qf)
    deltaJPos = qf - qi
    
    %% start search
    
    jPose = iiwa.getJointsPos();
    iiwa.sendJointsPositionsf(jPose);
    
    massOfTool=2.25; % the mass of the tool attached to flange in Kg
    cOMx=-50; % X coordinate of the center of mass of the tool in (mm)
    cOMy=0; % Y coordinate of the center of mass of the tool in (mm)
    cOMz=30; % Z coordinate of the center of mass of the tool in (mm)
    
    cStiffness = 100;
    rStiffness = 5;
    nStiffness = 0;
    
    iiwa.realTime_startImpedanceJoints(massOfTool,cOMx,cOMy,cOMz,...
    cStiffness,rStiffness,nStiffness);

    torque = [];
    A = 0;
    
    a=datevec(now);
    t0=a(6)+a(5)*60+a(4)*60*60; % calculate initial time

    dt=0;
    tstart=t0;

    time_stamps=[];

    
    while A <= 1;
    a=datevec(now);
    time=a(6)+a(5)*60+a(4)*60*60;
    dt=time-t0;
    limit = 0;
    jPosCommand = {qi(1)+A*deltaJPos(1) qi(2)+A*deltaJPos(2) qi(3)+A*deltaJPos(3) qi(4)+A*deltaJPos(4) qi(5)+A*deltaJPos(5) qi(6)+A*deltaJPos(6) qi(7)+A*deltaJPos(7)};
    jPosCommand{7} = qi(7)+ deg2rad(30)*sin(4*A*pi);
    A = A + 0.0005/2;
    temp = iiwa.sendJointsPositionsExTorque(jPosCommand);
    torque = [torque; temp{7}];
    time_stamps = [time_stamps; dt];
    if temp{7} > 0.5
        
        display('upper limit exceeded')
        limit = 1;
        break
    elseif temp{7} < -0.5
        
        display('lower limit exceeded')
        limit = 1;
        break
    end
    
    
    end
    
    display('done')
    
    %% move out of screw
    
    display('going up')
    
    jPose = iiwa.getJointsPos();
    iiwa.sendJointsPositionsf(jPose);
    qi = [jPose{1} jPose{2} jPose{3} jPose{4} jPose{5} jPose{6} jPose{7}]';
    dirkin = iiwa.gen_DirectKinematics(qi)

    temp = dirkin;
    temp(3,4) = dirkin(3,4)+0.05;
    
    lambda=0.01;
    n=500;
    qf = iiwa.gen_InverseKinematics( qi, temp,n,lambda )

    jPos = {qf(1) qf(2) qf(3) qf(4) qf(5) qf(6) qf(7)};
   
    %temp1 = iiwa.gen_DirectKinematics( qf(:,1))
%     for i = 1:7
    
    invkin = iiwa.gen_DirectKinematics(qf)
    deltaJPos = qf - qi
    A = 0;
    
    while A <= 1;
    jPosCommand = {qi(1)+A*deltaJPos(1) qi(2)+A*deltaJPos(2) qi(3)+A*deltaJPos(3) qi(4)+A*deltaJPos(4) qi(5)+A*deltaJPos(5) qi(6)+A*deltaJPos(6) qi(7)+A*deltaJPos(7)};
    jPosCommand{7} = qi(7)+ deg2rad(5)*sin(2*A*pi);
    A = A + 0.00005;
    iiwa.sendJointsPositionsf(jPosCommand);  
    end
    
%     pause(5); 
    
    iiwa.realTime_stopImpedanceJoints();
    end
    
    
    
catch e
    display(e)
    
end
%% final

iiwa.net_turnOffServer();

response = 0;
rosshutdown
