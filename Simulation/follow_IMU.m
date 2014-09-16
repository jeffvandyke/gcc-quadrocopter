function [] = follow_IMU()
% test 'plot_copter.m'!

% http://www.scribd.com/doc/96531606/Quadcopter-Math-Model-Amazing

  
% initial conditions

% persistent x xv y yv z zv roll rollv pitch pitchv yaw yawv;
%



figure
% motion paramaters

x = 0;
y = 0;
z = 0;

yaw = 0;
pitch = 25;
roll = 0;
    
%while(1)   
   
    
    rotate_copter(x,y,z,roll,pitch,yaw);
    
    pause(.004)
    
end




