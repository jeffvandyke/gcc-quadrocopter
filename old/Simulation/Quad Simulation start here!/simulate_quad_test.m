% test 'plot_copter.m' for speed!
% test paramaters
duration = 20; %  
step     = 0.1; %  
% initial conditions
x = 0;
y = 0;
z = 0;
roll  = 0;
pitch = 0;
yaw   = 0;

% motion paramaters

w = 300;
h = 300;
f = .25;


for t = 0:step:duration
 
    plot_copter(x,y,z,roll,pitch,yaw)
 
    x = exp(-.07*t)*300*cos(2*pi*f*t);
    y = exp(-.09*t)*500*sin(2*pi*f*t);
    z = exp(-.1*t)*400*sin(2*pi*.1*t);
    roll  = exp(-.075*t)*15*sin(2*pi*.5*t);
    pitch = exp(-.25*t)*10*sin(2*pi*.5*t);
    yaw   = exp(-.25*t)*90*sin(2*pi*.25*t);
    pause(.005)
    
end
