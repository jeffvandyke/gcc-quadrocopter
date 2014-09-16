function [quadc,history] = recordQuadSimulation()


[quadc,history] = initialize();

% motion paramaters
stop = false;
    % http://www.scribd.com/doc/96531606/Quadcopter-Math-Model-Amazing

    % test paramaters
    duration = 1000; %
    step     = .001; %
wait = step*30;
quadc.direction = quadc.alt - quadc.z;  % up:+, down:-

quadc.fig = figure('KeyPressFcn',@(obj,evt) key_detected(evt));
    function  [] = key_detected(evt)
        
        if strcmp(evt.Key,'uparrow')
            if quadc.alt < 1.9
                display('up')
                quadc.alt = quadc.alt + 0.2;
                quadc.direction = quadc.alt - quadc.z;
            end
        end
        if strcmp(evt.Key,'downarrow')
            if quadc.alt > .1
                display('down')
                quadc.alt = quadc.alt - 0.2;
                quadc.direction = quadc.alt - quadc.z;
            end
        end
        if strcmp(evt.Key,'a')
            display('left');
            quadc.left  = 1;
            quadc.right = 0;
        end
        if strcmp(evt.Key,'d')
            display('right');
            quadc.right = 1;
            quadc.left  = 0;
        end
        if strcmp(evt.Key,'w')
            display('back');
            quadc.back  = 1;
            quadc.front = 0;
        end
        if strcmp(evt.Key,'s')
            display('front');
            quadc.front = 1;
            quadc.back  = 0;
        end
        if strcmp(evt.Key,'space')
           display('SPACE!!')
           quadc.front = 0;
           quadc.back  = 0;
           quadc.right = 0;
           quadc.left  = 0;
           quadc.desiredTip = [0 0 1];
        end
        if strcmp(evt.Key,'escape')
            display('Ending');
            close(quadc.fig)
            stop = true;
            return;
        end
        if quadc.right
            quadc.desiredTip(1) = -0.05;
        end    
        if quadc.back
            quadc.desiredTip(2) = 0.05;
        end
        if quadc.front
            quadc.desiredTip(2) = -0.05;
        end
        if quadc.left
            quadc.desiredTip(1) = 0.05;
        end
        
        
    end


record_plot_copter(quadc.x,quadc.y,quadc.z,quadc.roll,quadc.pitch,quadc.yaw);

for t = 0:step:duration
    if stop
        break;
    end
   
if imag(quadc.x)
       display('Simulate broke X!')
       pause;
       return
end 

    quadc.t = t;
    % force vector:
    % F = [x y z]   x y and z are components of a unit vector such that
    % F = x*i + y*j + z*k and the magnitude of F is 1 (unit vector)
    

    n = [sind(quadc.roll)*cosd(quadc.pitch); sind(quadc.pitch)*cosd(quadc.roll); -cosd(quadc.pitch)*cosd(quadc.roll)];
    n = n/sqrt(n(1)^2+n(2)^2+n(3)^2);
    quadc.F = -n;

    
    if(~mod(t,.015)) % re-adjust every 15ms
        [quadc,history] = control_algorithm(quadc,history);
    end
   
    
    [quadc] = set_forces(quadc);
    F1 = quadc.f1*quadc.F;
    F2 = quadc.f2*quadc.F;
    F3 = quadc.f3*quadc.F;
    F4 = quadc.f4*quadc.F;
    
    % Forces: (converting from the quad-frame to the xyz-frame of reference)
    Fx = cosd(quadc.yaw)*(F1(1)+F2(1)+F3(1)+F4(1)) + sind(quadc.yaw)*(F1(2)+F2(2)+F3(2)+F4(2));
    Fy = sind(quadc.yaw)*(F1(1)+F2(1)+F3(1)+F4(1)) + cosd(quadc.yaw)*(F1(2)+F2(2)+F3(2)+F4(2));
    
    quadc.rollxyz = sind(quadc.yaw)*quadc.pitch + cosd(quadc.yaw)*quadc.roll;
    quadc.pitchxyz = sind(quadc.yaw)*quadc.roll + cosd(quadc.yaw)*quadc.pitch;
    
%   Acceleration and Velocities
%       Angular acc = torque/Angular accelleration
    quadc.rolla = (F1(3) - F3(3))/quadc.moment;
    quadc.rollv = quadc.rolla*step + quadc.rollv;
    quadc.roll  = quadc.rollv*step + quadc.roll;
    
% if t < 2
%     quadc.roll = 15*sin(2*pi*t);
% else
%     quadc.roll = 0;
% end
% 
% if t > 2 && t < 4
%     quadc.pitch = 15*sin(2*pi*t);
% else
%     quadc.pitch = 0;
% end
% 
% if t > 4 && t < 6
%     quadc.yaw = 15*sin(2*pi*t);
% else
%     quadc.yaw = 0;
% end
% 
% if t > 6 && t < 8
%     quadc.z = .5 + .25*sin(2*pi*t);
% else
%     quadc.z = 0.5;
% end
% if t > 8 
%     break
% end
% 

if t < 10
   quadc.yaw = 15*sin(pi*t);
   quadc.pitch = 15*sin(2*pi*t);
   quadc.roll = 20*sin(3*pi/2*t);
   quadc.z = .5+0.25*sin(.78*pi*t);
   quadc.x = .25*sin(3*pi/2*t);
   quadc.y = .25*sin(2*pi*t); 
end
    
%     
%     quadc.pitcha = (F2(3) - F4(3))/quadc.moment;
%     quadc.pitchv = quadc.pitcha*step + quadc.pitchv;
%     quadc.pitch  = quadc.pitchv*step + quadc.pitch;
%     
% %       Linear
%     quadc.xa = Fx/quadc.mass.total;
%     quadc.xv = quadc.xa*step + quadc.xv;
%     quadc.x  = quadc.xv*step + quadc.x;
    
%     quadc.ya = Fy/quadc.mass.total;
%     quadc.yv = quadc.ya*step + quadc.yv;
%     quadc.y  = quadc.yv*step + quadc.y;
%     
%     quadc.za = (F1(3)+F2(3)+F3(3)+F4(3)-quadc.weight)/quadc.mass.total;
%     quadc.zv = quadc.za*step + quadc.zv;
%     quadc.z  = quadc.zv*step + quadc.z;
    
    if(quadc.z < 0.010)  % quadrotor is on ground
        quadc = on_ground(quadc);
    end
    
   
     if(~mod(t,wait)) % plot every 1/30 calcs, 1:2 secs.
        quadc.aviobj = addframe(quadc.aviobj,gcf);       
        drawnow
        record_plot_copter(quadc.x,quadc.y,quadc.z,quadc.rollxyz,quadc.pitchxyz,quadc.yaw);
        pause(.0000001);
     end
end
quadc.aviobj = close(quadc.aviobj);
clear F F1 F2 F3 F4 Fx Fy duration step t wait
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [quadc, history] = initialize()
%% Initial Setting up (ignore)

%start video object thing
    quadc.aviobj = avifile('quadFlight.avi','compression','None');
    
    
    %Quad physical properties
    quadc.wingL       = 0.465;%meters
    quadc.wingH       = 0.02;% meters
    quadc.mass.motor  = 0.1;%kg
    quadc.mass.prop   = 0.05;%kg
    quadc.mass.center = 0.7;%kg
    quadc.mass.wing   = 0.05;%kg
    quadc.mass.total  = sum([quadc.mass.wing quadc.mass.motor quadc.mass.center]);
    quadc.weight      = quadc.mass.total*9.816;
    quadc.centerL     = 0.1;%meters
    quadc.centerH     = 0.035;%meters

    %moment of inertia | from attached wing   | from perpendicular wing
    % ch^2/12 + cl^2/12 + (mm*wl^2)/2 + (wm*wl)/6 + (wm*(wh^2 + ww^2))/12
    quadc.moment = 1/12*(quadc.centerL^2 + quadc.centerH^2 + 6*quadc.mass.motor*quadc.wingL^2 + 2*quadc.mass.wing*quadc.wingL + quadc.mass.wing*(quadc.wingH^2+quadc.wingL^2));

    % initial conditions
    quadc.t = 0;
    quadc.t1 = rand*360;
    quadc.t2 = rand*360;
    quadc.t3 = rand*360;
    quadc.t4 = rand*360;
    quadc.hover = 182;
    quadc.left  = 0;
    quadc.right = 0;
    quadc.front  = 0;
    quadc.back = 0;
    quadc.pwm1 = 127;
    quadc.pwm2 = 127;
    quadc.pwm3 = 127;
    quadc.pwm4 = 127;
        quadc.x  = 0;
        quadc.xv = 0;
        quadc.xa = 0;
        quadc.xi = 0;
        quadc.y  = 0;
        quadc.yv = 0;
        quadc.ya = 0;
        quadc.yi = 0;
    quadc.z  = 0.5; % start flying in the air.
        quadc.zv = 0;
        quadc.za = 0;
        quadc.zi = 0;
    quadc.roll = 0;
        quadc.rolli = quadc.roll;
        quadc.rolla  = 0;
        quadc.rollv  = 0;
    quadc.pitch = 0;
        quadc.pitchi = quadc.pitch;
        quadc.pitcha = 0;
        quadc.pitchv = 0;
    quadc.yaw   = 0;
        quadc.yawi = quadc.yaw;
        quadc.yawa   = 0;
        quadc.yawv   = 0;
    
    quadc.rollxyz = sind(quadc.yaw)*quadc.pitch + cosd(quadc.yaw)*quadc.roll;
    quadc.pitchxyz = sind(quadc.yaw)*quadc.roll + cosd(quadc.yaw)*quadc.pitch;
    
    quadc.desiredTip = [0,0,1]; % level
    quadc.rangePID = 2;

    quadc.alt = quadc.z;
    
    history.t = quadc.t;
    history.z = quadc.z;
    history.y = quadc.y;
    history.x = quadc.x;
    history.pwm1 = quadc.pwm1;
    history.pwm2 = quadc.pwm2;
    history.pwm3 = quadc.pwm3;
    history.pwm4 = quadc.pwm4;
    history.roll = quadc.roll;
    history.pitch = quadc.pitch;
    history.yaw = quadc.roll;
    history.xi = quadc.xi;
    history.yi = quadc.yi;
    history.zi = quadc.zi;
    history.rolli = quadc.rolli;
    history.pitchi = quadc.pitchi;
    history.yawi = quadc.yawi;
    
    history.pitchKP = 1.5;
    history.pitchKD = 3;
    history.pitchKI = 0;
    
    history.rollKP = history.pitchKP;
    history.rollKD = history.pitchKD;
    history.rollKI = history.pitchKI;
    
    
    history.zKP = 100;
    history.zKD = 100;
    history.zKI = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [quadc,history] = control_algorithm(quadc,history)

pitchChange = xyz2pitch(quadc.desiredTip) - quadc.pitchxyz;
rollChange = xyz2roll(quadc.desiredTip) - quadc.rollxyz;

quadc.xi = quadc.xi+quadc.x-0;
quadc.yi = quadc.yi+quadc.y-0;
quadc.zi = quadc.zi+quadc.z-quadc.alt;
quadc.rolli = quadc.rolli+rollChange;
quadc.pitchi = quadc.pitchi+pitchChange;
quadc.yawi = quadc.yawi+quadc.yaw;

% pitch ONLY

pitchinc = pitchChange*history.pitchKP - quadc.pitchv*history.pitchKD + history.pitchKI*quadc.pitchi;

if abs(pitchinc) > quadc.rangePID
    pitchinc = sign(pitchinc)*quadc.rangePID;
end





%%Roll ONLY

rollinc = rollChange*history.rollKP - quadc.rollv*history.rollKD + history.rollKI*quadc.rolli;

if abs(rollinc) > quadc.rangePID
    rollinc = sign(rollinc)*quadc.rangePID;
end


%% Height ONLY
zChange = quadc.alt - quadc.z;



zinc = zChange*history.zKP - quadc.zv*history.zKD - history.zKI*quadc.zi;

quadc.pwm1 = quadc.pwm1+rollinc + zinc;
quadc.pwm2 = quadc.pwm2+pitchinc + zinc;
quadc.pwm3 = quadc.pwm3-rollinc + zinc;
quadc.pwm4 = quadc.pwm4-pitchinc + zinc;

quadc = maxminPWM(quadc);
history = record(quadc,history);

end
function history = record(quadc,history)
    history.t = [history.t;quadc.t];
    history.z   = [history.z;quadc.z];
    history.y   = [history.y;quadc.y];
    history.x   = [history.x;quadc.x];
    history.roll = [history.roll;quadc.roll];
    history.pitch = [history.pitch;quadc.pitch];
    history.yaw = [history.yaw;quadc.yaw];
    history.pwm1 = [history.pwm1;quadc.pwm1];
    history.pwm2 = [history.pwm2;quadc.pwm2];
    history.pwm3 = [history.pwm3;quadc.pwm3];
    history.pwm4 = [history.pwm4;quadc.pwm4];
    history.xi = [history.xi;quadc.xi];
    history.yi = [history.yi;quadc.yi];
    history.zi = [history.zi;quadc.zi];
    history.rolli = [history.rolli;quadc.rolli];
    history.pitchi = [history.pitchi;quadc.pitchi];
    history.yawi = [history.yawi;quadc.yawi];
end
function [ pitch ] = xyz2pitch(xyz)
pitch = atand(xyz(2)/xyz(3));
end
function [ roll ] = xyz2roll(xyz)
roll = atand(xyz(1)/xyz(3));
end
function [quadc] = maxminPWM(quadc)

if quadc.pwm1<127
    quadc.pwm1 = 127;
elseif quadc.pwm1>255
    quadc.pwm1 = 255;
end
if quadc.pwm2<127
    quadc.pwm2 = 127;
elseif quadc.pwm2>255
    quadc.pwm2 = 255;
end
if quadc.pwm3<127
    quadc.pwm3 = 127;
elseif quadc.pwm3>255
    quadc.pwm3 = 255;
end
if quadc.pwm4<127
    quadc.pwm4 = 127;
elseif quadc.pwm4>255
    quadc.pwm4 = 255;
end
end


function [quadc] = on_ground(quadc)
    quadc.z = 0.01;
    quadc.zv = 0;
    quadc.za = 0;
    quadc.xv = 0;
    quadc.xa = -quadc.xa;
    quadc.yv = 0;
    quadc.ya = -quadc.ya;
    quadc.roll = 0;
    quadc.rollv = 0;
    quadc.rolla = 0;
    quadc.pitch = 0;
    quadc.pitchv = 0;
    quadc.pitcha = 0;
    quadc.left = 0;
    quadc.right = 0;
    quadc.back = 0;
    quadc.front = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [quadc] = set_forces(quadc)
    quadc.f1   = pwm2force(quadc.pwm1,quadc.t,quadc.t1);
    quadc.f2  = pwm2force(quadc.pwm2,quadc.t,quadc.t2);
    quadc.f3   = pwm2force(quadc.pwm3,quadc.t,quadc.t3);
    quadc.f4   = pwm2force(quadc.pwm4,quadc.t,quadc.t4);
    
end

function [force] = pwm2force(pwm,t,tdelay)
% The constant value is calculated from plots of our motors. (thrust vs. pwm)
% % load('test11_1.mat');
% % ans = LinearModel.fit(pwm11_1-127,thrustTOT11_1);
    
    amperr = 0.02; % Range Thrust
    freq = -5509*exp(-0.01347*pwm)+1083;    % frequcy of steady state force
    avg_thrust = .0003109*pwm^2-.06426*pwm+3.149; % average thrust
    
    force = avg_thrust + amperr*sind(freq*t+tdelay);
    if force <= 0
        force = 0;
    end
    
end