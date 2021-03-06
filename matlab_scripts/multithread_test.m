close all,clear all;clc;

try
rosinit( 'NodeName','matlab_multi_test');
catch e
    rosshutdown
    rosinit( 'NodeName','matlab_multi_test');
end


try
ip='172.31.1.147';
arg1=KST.LBR7R800; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
Tef_flange(3,4) = 0.153;
global iiwa;
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object
iiwa.net_establishConnection()
catch e
    display(e)
end

sub = rossubscriber("/joint_CB", "std_msgs/String", @robot_CB);



publisher = rospublisher('/joint_states', 'sensor_msgs/JointState');
rosmsg = rosmessage('sensor_msgs/JointState');
rosmsg.Header.Stamp = rostime("now");

rosmsg.Name = {'iiwa_joint_1' 'iiwa_joint_2' 'iiwa_joint_3' 'iiwa_joint_4' 'iiwa_joint_5' 'iiwa_joint_6' 'iiwa_joint_7'};
rosmsg.Velocity = [0 0 0 0 0 0 0];


state = iiwa.getJointsPos();
rosmsg.Position = [state{1} state{2} state{3} state{4} state{5} state{6} state{7}];
rosmsg.Header.Stamp = rostime("now");
send(publisher, rosmsg);


iiwa.net_turnOffServer()
rosshutdown