close all,clear all;clc;

% node = ros.Node("matlab_robot_CB_test");

ip='172.31.1.147'; % The IP of the controller
% start a connection with the server
global t_Kuka;
t_Kuka=net_establishConnection( ip );

if ~exist('t_Kuka','var') || isempty(t_Kuka) || strcmp(t_Kuka.Status,'closed')
  warning('Connection could not be establised, script aborted');
  return;
else
    %% Create the robot object
    arg1=KST.LBR7R800; % choose the robot iiwa7R800 or iiwa14R820
    arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
    Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
    global iiwa;
    iiwa=KST('',arg1,arg2,Tef_flange); % create the object
    
    
    try
%     jPos={0,0,0,pi/2,0,-pi/2,0};  
%     relVel=0.15;     
%     movePTPJointSpace( t_Kuka , jPos, relVel);
    
%     sub = ros.Subscriber(node, "/CB_test", "std_msgs/String", @robot_CB);
    pause(600)
    
    catch
        
        fprintf("error\n")
        net_turnOffServer( t_Kuka );
        fclose(t_Kuka);
        rosshutdown;
        return;
    end
    
    rosshutdown;
    net_turnOffServer( t_Kuka );
    fclose(t_Kuka);
end



function robot_CB(sub, msg)

    temp = strsplit(msg.Data, "_");
    global realTime;

    if temp{1} == "Cartesian"
        if realTime == true
            fprintf("\nUse command: RealTimeCartesian")
            return
        end
        fprintf("\nPlanning for Cartesian pose\n\n")
        pose = zeros(6,1);
        for i=1:6
            pose(i) = str2num(temp{i+2});
        end
        velocity = str2num(temp{10});

        fprintf("Cartesian pose:")
        disp(pose)
        fprintf("Velocity:")
        disp(velocity)

        pose = {pose(1) pose(2) pose(3) pose(4) pose(5) pose(6)};
        
        global t_Kuka;
        movePTPLineEEF(t_Kuka, pose, velocity);
        
    elseif temp{1} == "JointSpace"
        if realTime == true
            fprintf("\nUse command: realTimeJointSpace")
            return
        end
        fprintf("\nPlanning for JointSpace movement\n\n")
        pose = zeros(7,1);
        for i=1:7
            pose(i) = str2num(temp{i+2});
        end
        velocity = str2num(temp{11});
        
        fprintf("Joint pose:")
        disp(pose)
        fprintf("Velocity:")
        disp(velocity)
            
        pose = {pose(1) pose(2) pose(3) pose(4) pose(5) pose(6) pose(7)};
        
        global t_Kuka;
        movePTPJointSpace( t_Kuka, pose, velocity);
        
    elseif temp{1} == "StartImpedance" 
        fprintf("\nInitiating impedance control")
        global t_Kuka;
        
        stiffness= zeros(3,1);
        for i=1:3
            stiffness(i) = str2num(temp{i+2});
        end
        startImpedanceControl( t_Kuka, stiffness(1), stiffness(2), stiffness(3));
        realTime = true;
        
    elseif temp{1} == "StopImpedance"
        fprintf("\nStopping impedance control")
        global t_Kuka;
        
        realTime_stopImpedanceJoints( t_Kuka);
        realTime = false;
        
    elseif temp{1} == "RealTimeCartesian"
        if realTime == false
            fprintf("Realtime control not enabled")
            return
        end
        fprintf("Planning Cartesian movement using realtime control")
        pose = [0 0 0 0 0 0];
        for i=1:6
            pose(i) = str2num(temp{i+2});
        end
        velocity = str2num(temp{10});

        fprintf("Cartesian pose:")
        disp(pose)
        fprintf("Velocity:")
        disp(velocity)
        
        global t_Kuka;
        realtime_moveCartesian(t_Kuka, pose, 1);
      
        
    elseif temp{1} == "RealTimeRelativeCartesian"
        if realTime == false
            fprintf("Realtime control not enabled")
            return
        end
        fprintf("Planning relative Cartesian movement using realtime control")
        pose = [0 0 0 0 0 0];
        for i=1:6
            pose(i) = str2num(temp{i+2});
        end
        velocity = str2num(temp{10});

        fprintf("Cartesian pose:")
        disp(pose)
        fprintf("Velocity:")
        disp(velocity)
        
        global t_Kuka;
        realtime_relativeMoveCartesian(t_Kuka, pose, 1);
    end

end


function [ ret ] = startImpedanceControl( robot, cStiffness, rStiffness, nStiffness)
    massOfTool=0.5; % the mass of the tool attached to flange in Kg
    cOMx=0; % X coordinate of the center of mass of the tool in (mm)
    cOMy=0; % Y coordinate of the center of mass of the tool in (mm)
    cOMz=40; % Z coordinate of the center of mass of the tool in (mm)

    if rStiffness>=300
        rStiffness=300;
    end
    if nStiffness>=200
        nStiffness=200;
    end

    % Start the realtime control with impedence
    ret = realTime_startImpedanceJoints(robot,massOfTool,cOMx,cOMy,cOMz,...
    cStiffness,rStiffness,nStiffness);
    
    if ret==false
        disp('Acknowledge, realtime control with impedance failed');
    end
end

function [eefPose, jPose] = getRobotPosition( robot )
     jPose = getJointsPos(robot);
     global realTime
     if realTime == false
         eefPose = getEEFPos( robot );
     else
         eefPose = [];
     end
end

function [ state ] = realtime_moveCartesian( robot, pose, vel )
    global iiwa;
    currentJPos = getJointsPos(robot);

    q = [currentJPos{1};currentJPos{2};currentJPos{3};currentJPos{4};currentJPos{5};currentJPos{6};currentJPos{7}];

    R = makehgtform('xrotate', pose(4), 'yrotate', pose(5), 'zrotate', pose(6));
    goalTransform = R + [ 0    0    0   pose(1)/1000
                          0    0    0   pose(2)/1000
                          0    0    0   pose(3)/1000
                          0    0    0   1.0000];
                      
    lambda=0.01;
    n=500;
    qs = iiwa.gen_InverseKinematics( q, goalTransform,n,lambda );

    
    jPos = {qs(1) qs(2) qs(3) qs(4) qs(5) qs(6) qs(7)};

    deltaJPos = qs - q;
    disp(deltaJPos)
    A = 0;
    while A <= 1;
    jPosCommand = {q(1)+A*deltaJPos(1) q(2)+A*deltaJPos(2) q(3)+A*deltaJPos(3) q(4)+A*deltaJPos(4) q(5)+A*deltaJPos(5) q(6)+A*deltaJPos(6) q(7)+A*deltaJPos(7)};
    A = A + 0.0001*vel;
    sendJointsPositionsf( robot ,jPosCommand);
    end
    pause(0.5);
    
    pose_test = getJontsPos(robot)
    q = iiwa.gen_DirectKinematics(pose_test)
    goalTransform-q
end

function [ state ] = realtime_relativeMoveCartesian( robot, delta, vel )
    global iiwa;
    currentJPos = getJointsPos(robot);

    q = [currentJPos{1};currentJPos{2};currentJPos{3};currentJPos{4};currentJPos{5};currentJPos{6};currentJPos{7}];
    
    currentTransform = iiwa.gen_DirectKinematics(q)
    currentR = currentTransform(1:3,1:3);
    currentR = [currentR [0;0;0];[0 0 0 1]];
    
    
    R = makehgtform('xrotate', delta(4), 'yrotate', delta(5), 'zrotate', delta(6))

    deltaR = currentR*R;
    
    goalTransform = deltaR + [ 0    0    0   currentTransform(1,4)+delta(1)/1000
                                0    0    0   currentTransform(2,4)+delta(2)/1000
                                0    0    0   currentTransform(3,4)+delta(3)/1000
                                0    0    0   1.0000];
    
    disp(goalTransform)
                      
    lambda=0.01;
    n=500;
    qs = iiwa.gen_InverseKinematics( q, goalTransform,n,lambda );

    jPos = {qs(1) qs(2) qs(3) qs(4) qs(5) qs(6) qs(7)};

    deltaJPos = qs - q;
    disp(deltaJPos)
    A = 0;
    while A <= 1;
    jPosCommand = {q(1)+A*deltaJPos(1) q(2)+A*deltaJPos(2) q(3)+A*deltaJPos(3) q(4)+A*deltaJPos(4) q(5)+A*deltaJPos(5) q(6)+A*deltaJPos(6) q(7)+A*deltaJPos(7)};
    A = A + 0.0001*vel;
    sendJointsPositionsf( robot ,jPosCommand);
    end
    pause(0.5);
end

