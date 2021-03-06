close all,clear all;clc;

node = ros.Node("matlab_CB_test");

try
sub = ros.Subscriber(node, "/CB_test", "std_msgs/String", @test_CB);


pause(60)

catch
    fprintf("error\n")
rosshutdown
return
end
rosshutdown



function test_CB(sub, msg)

    disp(msg.Data)

end

