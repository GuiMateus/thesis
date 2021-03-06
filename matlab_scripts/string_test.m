

close all,clear all;clc;


node = ros.Node("matlab_string_test");

sub = ros.Subscriber(node, "/string_test", "std_msgs/String")

ip='172.31.1.147'; % The IP of the controller
% start a connection with the server
global t_Kuka;
t_Kuka=net_establishConnection( ip );

if ~exist('t_Kuka','var') || isempty(t_Kuka) || strcmp(t_Kuka.Status,'closed')
  warning('Connection could not be establised, script aborted');
  return;
else
    try
        for counter=1:2
        test = receive(sub);
        disp(test)
        %test = "Cartesian_pose:_-300_150_500_pi_0_-pi_vel:_0.15";

        temp = strsplit(test.Data, "_");


        if temp{1} == "Cartesian"
            fprintf("Planning for Cartesian pose\n\n")
            pose = [];
            for i=3:8
                pose = [pose, str2num(temp{i})];
            end
            velocity = str2num(temp{10});

            fprintf("Cartesian pose:")
            disp(pose)
            fprintf("Velocity:")
            disp(velocity)
            
            pose = {pose(1) pose(2) pose(3) pose(4) pose(5) pose(6)};

            movePTPLineEEF(t_Kuka, pose, velocity);
        end
        end
    catch
        net_turnOffServer( t_Kuka );
        fclose(t_Kuka);
        rosshutdown;
        return;
    end
    rosshutdown;
    net_turnOffServer( t_Kuka );
    fclose(t_Kuka);
end