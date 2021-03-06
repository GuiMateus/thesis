%% Example
% real time control of the KUKA iiwa 7 R 800
% the impedence control is turned on
% Moving first joint of the robot, using a sinisoidal function

% An example script, it is used to show how to use the different
% functions of the KUKA Sunrise matlab toolbox

% First start the server on the KUKA iiwa controller
% Then run this script using Matlab

% Copyright: Mohammad SAFEEA, 8th of Nov 2017

% Important: Be careful when runnning the script, be sure that no human, nor obstacles
% are around the robot

close all,clear all;clc;
already_closed = 0;
warning('off')

ip='172.31.1.147'; % The IP of the controller
% start a connection with the server
global t_Kuka;
t_Kuka=net_establishConnection( ip );

if ~exist('t_Kuka','var') || isempty(t_Kuka) || strcmp(t_Kuka.Status,'closed')
  warning('Connection could not be establised, script aborted');
  return;
else
    
    %% Get position and orientation of end-effector

    moveRobotInitPose(t_Kuka);

    [testPose, currentJPos] = getRobotPosition(t_Kuka);

    %% Create the robot object
    arg1=KST.LBR7R800; % choose the robot iiwa7R800 or iiwa14R820
    arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
    Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
    iiwa=KST('',arg1,arg2,Tef_flange); % create the object

    
    %% Moving using cartesian coordinates in impedance mode
    startImpedanceControl(t_Kuka, 300, 50, 200);

    realtime_moveCartesian(iiwa, t_Kuka, currentJPos,[-0.3 -0.25 0.5 pi 0 pi], 1)

    realTime_stopImpedanceJoints( t_Kuka );

    %% Move 4th joint in impedance control
    try
        [testPose, currentJPos] = getRobotPosition(t_Kuka);
        startImpedanceControl(t_Kuka, 300, 50, 200);
        w=0.6; % motion constants, frequency rad/sec
        A=0.2; % motion constants, amplitude of motion
        dt=0;
        counter=0;
        duration=1*10; %1 minutes
        a=datevec(now);
        time_stamps=zeros(1,1000*duration);
        t0=a(6)+a(5)*60+a(4)*60*60; % calculate initial time

        precission=1000;
        precission_flag=true;
        jPosCommand = currentJPos;
        PoseCommand = testPose;
        while(dt<duration)
            % perform trajectory calculation here
            a=datevec(now);
            time=a(6)+a(5)*60+a(4)*60*60;
            dt=time-t0;

            temp=A*(1-cos(w*dt));
            temp2 = 0.2*(1-cos(0.6*dt));
            jPosCommand{5}=currentJPos{5}+temp2;
            PoseCommand{2}=testPose{2}+temp;

            counter=counter+1;
            % Send joint positions to robot
            sendJointsPositionsf( t_Kuka ,jPosCommand);
            %           sendEEfPositionf( t_Kuka ,PoseCommand);
            time_stamps(counter)=dt;

        end
    catch

        fprintf('I fucked up')
        % Stop the realtime control with impedence motion
        realTime_stopImpedanceJoints( t_Kuka );
        % turn off the server
        net_turnOffServer( t_Kuka );


        fclose(t_Kuka);
    already_closed = 1;
    end 

    if already_closed == 0
        % Stop the realtime control with impedence motion
        realTime_stopImpedanceJoints( t_Kuka );

        % turn off the server
        net_turnOffServer( t_Kuka );


        fclose(t_Kuka);
        return
    end
end
warning('on')
 
function [ state ]=moveRobotInitPose( robot )

      jPos={0,0,0,pi/2,0,-pi/2,0};  
      relVel=0.15;     
      state = movePTPJointSpace( robot , jPos, relVel);
end

function [eefPose, jPose] = getRobotPosition( robot )
     jPose = getJointsPos(robot);
     eefPose = getEEFPos( robot );
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

function [ state ] = realtime_moveCartesian( rClass, robot, currentJPos, pose, vel )


    q = [currentJPos{1};currentJPos{2};currentJPos{3};currentJPos{4};currentJPos{5};currentJPos{6};currentJPos{7}]

    R = makehgtform('xrotate', pose(4), 'yrotate', pose(5), 'zrotate', pose(6));
    goalTransform = R + [ 0    0    0   pose(1)
                          0    0    0   pose(2)
                          0    0    0   pose(3)
                          0    0    0   1.0000];
                      
    lambda=0.01;
    n=500;
    qs=rClass.gen_InverseKinematics( q, goalTransform,n,lambda )

    jPos = {qs(1) qs(2) qs(3) qs(4) qs(5) qs(6) qs(7)};

    deltaJPos = qs - q;
    A = 0;
    while A <= 1;
    jPosCommand = {q(1)+A*deltaJPos(1) q(2)+A*deltaJPos(2) q(3)+A*deltaJPos(3) q(4)+A*deltaJPos(4) q(5)+A*deltaJPos(5) q(6)+A*deltaJPos(6) q(7)+A*deltaJPos(7)};
    A = A + 0.0001*vel;
    sendJointsPositionsf( robot ,jPosCommand);
    end
    pause(0.5)
    
    currentJPos = getJointsPos(robot);
    
    delta = [currentJPos{1} - jPosCommand{1}
    currentJPos{2} - jPosCommand{2}
    currentJPos{3} - jPosCommand{3}
    currentJPos{4} - jPosCommand{4}
    currentJPos{5} - jPosCommand{5}
    currentJPos{6} - jPosCommand{6}
    currentJPos{7} - jPosCommand{7}]

end
