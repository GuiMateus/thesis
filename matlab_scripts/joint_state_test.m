ip='172.31.1.147';

arg1=KST.LBR7R800; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
Tef_flange(3,4) = 0.153;
global iiwa;
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object
iiwa.net_establishConnection()


try

  pose = iiwa.getEEFPos()

%  massOfTool=2.38 % the mass of the tool attached to flange in Kg
% cOMx=-20; % X coordinate of the center of mass of the tool in (mm)
% cOMy=0; % Y coordinate of the center of mass of the tool in (mm)
% cOMz=30; % Z coordinate of the center of mass of the tool in (mm)
% 
% cStiffness = 50;
% rStiffness = 300;
% nStiffness = 0;
% 
% beforejPose = iiwa.getJointsPos();
% iiwa.sendJointsPositionsf(beforejPose);
% beforePose = iiwa.getEEFPos();
% 
% iiwa.realTime_startImpedanceJoints(massOfTool,cOMx,cOMy,cOMz,...
% cStiffness,rStiffness,nStiffness);
% 
% pause(5)
% iiwa.realTime_stopImpedanceJoints();
catch e
    display(e)
end

iiwa.net_turnOffServer();
