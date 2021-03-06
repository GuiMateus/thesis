close all,clear all;clc;

%node = ros.Node("matlab_robot_CB_test");

ip='172.31.1.147'; % The IP of the controller
% start a connection with the server
global t_Kuka;
% t_Kuka=net_establishConnection( ip );
% 
% if ~exist('t_Kuka','var') || isempty(t_Kuka) || strcmp(t_Kuka.Status,'closed')
%   warning('Connection could not be establised, script aborted');
%   return;
% else
    %% Create the robot object
    arg1=KST.LBR7R800; % choose the robot iiwa7R800 or iiwa14R820
    arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
    Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
    Tef_flange(3,4) = 0.153;
    global iiwa;
    iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object
    iiwa.net_establishConnection()
    
    
    
    
    % -311 308.5 325 2.55 0 -3.1414 screw position? at least it's close.
    % joint position {[1.7702]}    {[0.1631]}    {[0.6627]}    {[-1.7106]}    {[-0.1036]}    {[1.3032]}
    % almost close enough
    
    % {[1.2455]}    {[0.4360]}    {[1.4599]}    {[-1.6690]}    {[-0.4348]}    {[1.4382]}    {[0.3983]}
    % {[2.2205]}    {[0.1167]}    {[0.1521]}    {[-1.6731]}    {[-0.0190]}    {[1.3533]}    {[0.0228]}
    temp1 = {1.2455 0.4360 1.4599 -1.6690 -0.4348 1.4382 0.3983};
    temp2 = {2.2205 0.1167 0.1521 -1.6731 -0.0190 1.3533 0.0228};
    
%     
%     jPose1 = iiwa.getJointsPos()
%     jPose = {pi/2 0 0 -pi/2 0 pi/2 0};
%     pose = {-309 308.5 400 -0.7872 0 -3.1414};
% 
%     iiwa.movePTPLineEEF(pose, 10);
%     
%     
%     pose = {-309 308.5 340 -0.7872 0 -3.1414};
%     
%     joints_indices=[3 4]; % joint index start from one
%     max_torque=[1 2];
%     min_torque=[-1 -8];
%     
%     temp = iiwa.getJointsMeasuredTorques()
%     
%     stopped = iiwa.movePTP_ConditionalTorque_LineEEF(pose, 10, joints_indices, max_torque, min_torque)
%     
%     temp = iiwa.getJointsMeasuredTorques()
% 
%     jPose = iiwa.getJointsPos();
%     iiwa.sendJointsPositionsf(jPose);
%     qi = [jPose{1} jPose{2} jPose{3} jPose{4} jPose{5} jPose{6} jPose{7}]';
%     dirkin = iiwa.gen_DirectKinematics(qi)
% 
%     temp = dirkin;
%     temp(3,4) = dirkin(3,4)-0.01;
%     
%     lambda=0.01;
%     n=500;
%     qf = iiwa.gen_InverseKinematics( qi, temp,n,lambda )
% 
%     jPos = {qf(1) qf(2) qf(3) qf(4) qf(5) qf(6) qf(7)};
%    
%     %temp1 = iiwa.gen_DirectKinematics( qf(:,1))
% %     for i = 1:7
%     
%     invkin = iiwa.gen_DirectKinematics(qf)
%     deltaJPos = qf - qi
%     
    jPose = iiwa.getJointsPos();
    iiwa.sendJointsPositionsf(jPose);
    
    massOfTool=2.3; % the mass of the tool attached to flange in Kg
    cOMx=-50; % X coordinate of the center of mass of the tool in (mm)
    cOMy=0; % Y coordinate of the center of mass of the tool in (mm)
    cOMz=30; % Z coordinate of the center of mass of the tool in (mm)
    
    cStiffness = 50;
    rStiffness = 5;
    nStiffness = 0;
    
    iiwa.realTime_startImpedanceJoints(massOfTool,cOMx,cOMy,cOMz,...
    cStiffness,rStiffness,nStiffness);

    pause(10)

%     torque = [];
%     A = 0;
%     
%     a=datevec(now);
%     t0=a(6)+a(5)*60+a(4)*60*60; % calculate initial time
% 
%     dt=0;
%     tstart=t0;
% 
%     time_stamps=[];
%     
%     jPoseInit = iiwa.getJointsPos();
%     
%     while A <= 1;
%     a=datevec(now);
%     time=a(6)+a(5)*60+a(4)*60*60;
%     dt=time-t0;
%     limit = 0;
%     jPosCommand = {qi(1)+A*deltaJPos(1) qi(2)+A*deltaJPos(2) qi(3)+A*deltaJPos(3) qi(4)+A*deltaJPos(4) qi(5)+A*deltaJPos(5) qi(6)+A*deltaJPos(6) qi(7)+A*deltaJPos(7)};
%     jPosCommand{7} = qi(7)+ deg2rad(30)*sin(2*A*pi);
%     A = A + 0.0005;
%     temp = iiwa.sendJointsPositionsExTorque(jPosCommand);
%     torque = [torque; temp{7}];
%     time_stamps = [time_stamps; dt];
%     if temp{7} > 0.5
%         
%         display('upper limit exceeded')
%         limit = 1;
%         break
%     elseif temp{7} < -0.5
%         
%         display('lower limit exceeded')
%         limit = 1;
%         break
%     end
%     
%     
%     end
%     
%     display('done')
%     
%     display('going up')
%     
%     jPose = iiwa.getJointsPos();
%     angle = rad2deg(jPose{7});
%     iiwa.sendJointsPositionsf(jPose);
%     qi = [jPose{1} jPose{2} jPose{3} jPose{4} jPose{5} jPose{6} jPose{7}]';
%     dirkin = iiwa.gen_DirectKinematics(qi)
% 
%     temp = dirkin;
%     temp(3,4) = dirkin(3,4)+0.05;
%     
%     lambda=0.01;
%     n=500;
%     qf = iiwa.gen_InverseKinematics( qi, temp,n,lambda )
% 
%     jPos = {qf(1) qf(2) qf(3) qf(4) qf(5) qf(6) qf(7)};
%    
%     %temp1 = iiwa.gen_DirectKinematics( qf(:,1))
% %     for i = 1:7
%     
%     invkin = iiwa.gen_DirectKinematics(qf)
%     deltaJPos = qf - qi
%     A = 0;
%     
%     while A <= 1;
%     jPosCommand = {qi(1)+A*deltaJPos(1) qi(2)+A*deltaJPos(2) qi(3)+A*deltaJPos(3) qi(4)+A*deltaJPos(4) qi(5)+A*deltaJPos(5) qi(6)+A*deltaJPos(6) qi(7)+A*deltaJPos(7)};
%     jPosCommand{7} = qi(7)+ deg2rad(5)*sin(2*A*pi);
%     A = A + 0.00005;
%     iiwa.sendJointsPositionsf(jPosCommand);  
%     end
%     
% %     pause(5); 
    
    iiwa.realTime_stopImpedanceJoints();

    jPoseFin = iiwa.getJointsPos();
    
    iiwa.net_turnOffServer();

%     plot(time_stamps, torque)
    %iiwa.fclose();
%     display(limit)
%     display(angle)
% end