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

pubJoint = rospublisher('/joint_temp', 'sensor_msgs/JointState');
pubTrigger = rospublisher('/matlab_trigger', 'std_msgs/Empty');

screwPos = rossubscriber('/vision_positions_pc', 'sensor_msgs/PointCloud');


jointmsg = rosmessage(pubJoint);
Joint_names = {'iiwa_joint_1' 'iiwa_joint_2' 'iiwa_joint_3' 'iiwa_joint_4' 'iiwa_joint_5' 'iiwa_joint_6' 'iiwa_joint_7'};
vel = [0 0 0 0 0 0 0];

jointmsg.Name = Joint_names;
jointmsg.Velocity = vel;


try
    init_jPos = {-0.1315 0.6162 -0.1544 -1.3325 0.6042 1.3900 1.1360};
    init_jPos2 = {0.6976 1.0010 -0.1545 -0.7102 -0.3017 1.7334 2.1607};
    
    pose = {-309 308.5 370 -0.7872 0 -3.1414};
    iiwa.movePTPJointSpace(init_jPos, 0.15);
%     iiwa.movePTPLineEEF(pose, 20);
    
    
    temp_joint_pose = iiwa.getJointsPos();
    temp_JPos = [temp_joint_pose{1} temp_joint_pose{2} temp_joint_pose{3} temp_joint_pose{4}];
    temp_JPos = [temp_JPos temp_joint_pose{5} temp_joint_pose{6} temp_joint_pose{7}];
    
    jointmsg.Position = temp_JPos;
    jointmsg.Header.Stamp = rostime("now");
    
    send(pubJoint, jointmsg);
    empty = rosmessage('std_msgs/Empty');
    send(pubTrigger, empty);
    
    deltaPos = [];
    deltaPos2 = [];
    screwResp = receive(screwPos, 10)
    
%     display(screwResp.Points(1))

    for i = 1:length(screwResp.Points)
        temp_pos = screwResp.Points(i);
        pose = {temp_pos.X*1000 36 190    pi/2 0 pi};
        
        iiwa.movePTPLineEEF(pose, 20);
        
        pose = {temp_pos.X*1000-3 temp_pos.Y*1000+3 110    pi/2 0 pi};
        
        joints_indices=[3 4]; % joint index start from one
        max_torque=[10 10];
        min_torque=[-10 -10];

        stopped = iiwa.movePTP_ConditionalTorque_LineEEF(pose, 20, joints_indices, max_torque, min_torque);
        display(stopped)
        
        %% gui wanted peg in hole
        
        
        %% I wanted impedance
        
        massOfTool=2.38; % the mass of the tool attached to flange in Kg
        cOMx=-20; % X coordinate of the center of mass of the tool in (mm)
        cOMy=0; % Y coordinate of the center of mass of the tool in (mm)
        cOMz=30; % Z coordinate of the center of mass of the tool in (mm)

        cStiffness = 50;
        rStiffness = 300;
        nStiffness = 0;
    
        beforejPose = iiwa.getJointsPos();
        iiwa.sendJointsPositionsf(beforejPose);
        beforePose = iiwa.getEEFPos();
        
        iiwa.realTime_startImpedanceJoints(massOfTool,cOMx,cOMy,cOMz,...
        cStiffness,rStiffness,nStiffness);

        pause(5)
        
        iiwa.realTime_stopImpedanceJoints();
        
        afterPose = iiwa.getEEFPos();
        
        deltaPos = [deltaPos; i beforePose{1}-afterPose{1} beforePose{2}-afterPose{2}];
        
        iiwa.realTime_startImpedanceJoints(massOfTool,cOMx,cOMy,cOMz,...
        cStiffness,rStiffness,nStiffness);

        pause(5);
        
        iiwa.realTime_stopImpedanceJoints();
        
        
        
        
        %% moving to abovee pump
        
        pose = {temp_pos.X*1000 36 190  pi/2 0 pi};
        
        iiwa.movePTPLineEEF(pose, 20);
    end
    
    a = 0;
    
    %% second iteration
    
   iiwa.movePTPJointSpace(init_jPos2, 0.15);
%     iiwa.movePTPLineEEF(pose, 20);
    
    
    temp_joint_pose = iiwa.getJointsPos();
    temp_JPos = [temp_joint_pose{1} temp_joint_pose{2} temp_joint_pose{3} temp_joint_pose{4}];
    temp_JPos = [temp_JPos temp_joint_pose{5} temp_joint_pose{6} temp_joint_pose{7}];
    
    jointmsg.Position = temp_JPos;
    jointmsg.Header.Stamp = rostime("now");
    
    send(pubJoint, jointmsg);
    empty = rosmessage('std_msgs/Empty');
    send(pubTrigger, empty);
    
    
    screwResp = receive(screwPos, 10)
    
%     display(screwResp.Points(1))

    for i = 1:length(screwResp.Points)
        temp_pos = screwResp.Points(i);
        pose = {temp_pos.X*1000 200 190    pi/2 0 pi};
        
        iiwa.movePTPLineEEF(pose, 20);
        
        pose = {temp_pos.X*1000-3 temp_pos.Y*1000+3 110    pi/2 0 pi};
        
        joints_indices=[3 4]; % joint index start from one
        max_torque=[10 10];
        min_torque=[-10 -10];

        stopped = iiwa.movePTP_ConditionalTorque_LineEEF(pose, 20, joints_indices, max_torque, min_torque);
        display(stopped)

        %% gui wanted peg in hole
        
        
        %% I wanted impedance
        
        massOfTool=2.38; % the mass of the tool attached to flange in Kg
        cOMx=-20; % X coordinate of the center of mass of the tool in (mm)
        cOMy=0; % Y coordinate of the center of mass of the tool in (mm)
        cOMz=30; % Z coordinate of the center of mass of the tool in (mm)

        cStiffness = 50;
        rStiffness = 300;
        nStiffness = 0;
    
        beforejPose = iiwa.getJointsPos();
        iiwa.sendJointsPositionsf(beforejPose);
        beforePose = iiwa.getEEFPos();
        
        iiwa.realTime_startImpedanceJoints(massOfTool,cOMx,cOMy,cOMz,...
        cStiffness,rStiffness,nStiffness);

        pause(5)
        
        iiwa.realTime_stopImpedanceJoints();
        
        afterPose = iiwa.getEEFPos();
        
        deltaPos2 = [deltaPos2; i beforePose{1}-afterPose{1} beforePose{2}-afterPose{2}];
        
        iiwa.realTime_startImpedanceJoints(massOfTool,cOMx,cOMy,cOMz,...
        cStiffness,rStiffness,nStiffness);

        pause(5);
        
        iiwa.realTime_stopImpedanceJoints();
        
        
        
        
        %% moving to abovee pump
        
        pose = {temp_pos.X*1000 200 190    pi/2 0 pi};
        
        iiwa.movePTPLineEEF(pose, 20);
    end
    
    
    
catch e
    display(e)
    
end

display(deltaPos)
display(deltaPos2)

iiwa.net_turnOffServer();

response = 0;
rosshutdown
