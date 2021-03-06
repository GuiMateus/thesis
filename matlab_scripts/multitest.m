
e = 0;
try
    rosinit( 'NodeName','matlab_multiworker_test');
catch e
    rosshutdown
    rosinit( 'NodeName','matlab_multiworker_test');
end



try
triggerclient = rossvcclient('/matlab_test_service');
temp = rosmessage('std_srvs/EmptyRequest');


% used to initiate the service connection, for some reason this has to
% happen before the real connection can be used
for i = 1:2
try
    test = triggerclient.call(temp)
end 
end
try
    test = triggerclient.call(temp)
end 


catch e
    display(e)
end
rosshutdown