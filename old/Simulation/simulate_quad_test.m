function [quadc] = simulate_quad_test()


quadc = initialize();

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
        if strcmp(evt.Key,'escape')
            display('Ending');
            close(quadc.fig)
            stop = true;
            return;
        end
        
    end


rotate_copter(quadc.x,quadc.y,quadc.z,quadc.roll,quadc.pitch,quadc.yaw);

for t = 0:step:duration
    if stop
        break;
    end
    quadc.t = t;
    % force vector:
    % F = [x y z]   x y and z are components of a unit vector such that
    % F = x*i + y*j + z*k and the magnitude of F is 1 (unit vector)
    n = [sind(quadc.roll)*cosd(quadc.pitch); sind(quadc.pitch)*cosd(quadc.roll); -cosd(quadc.pitch)*cosd(quadc.roll)];
    n = n/sqrt(n(1)^2+n(2)^2+n(3)^2);
    quadc.F = -n;
    
    [quadc] = control_algorithm(quadc);
    
    
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
    
%     quadc.pitcha = (F2(3) - F4(3))/quadc.moment;
%     quadc.pitchv = quadc.pitcha*step + quadc.pitchv;
%     quadc.pitch  = quadc.pitchv*step + quadc.pitch;
%     
%       Linear
%     quadc.xa = Fx/quadc.mass.total;
%     quadc.xv = quadc.xa*step + quadc.xv;
%     quadc.x  = quadc.xv*step + quadc.x;
%     
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
        rotate_copter(quadc.x,quadc.y,quadc.z,quadc.rollxyz,quadc.pitchxyz,quadc.yaw);
        pause(.0000001);
     end
end
clear F F1 F2 F3 F4 Fx Fy duration step t wait
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [quadc] = initialize()
%% Initial Setting up (ignore)

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
    quadc.hover = 127;
    quadc.left  = 0;
    quadc.right = 0;
    quadc.front  = 0;
    quadc.back = 0;
    quadc.pwm1 = 0;
    quadc.pwm2 = 0;
    quadc.pwm3 = 0;
    quadc.pwm4 = 0;
        quadc.x  = 0;
        quadc.xv = 0;
        quadc.xa = 0;
        quadc.y  = 0;
        quadc.yv = 0;
        quadc.ya = 0;
    quadc.z  = 0; % start flying in the air.
        quadc.zv = 0;
        quadc.za = 0;
        quadc.rolla  = 0;
        quadc.rollv  = 0;
    quadc.roll = 0;
        quadc.pitcha = 0;
        quadc.pitchv = 0;
    quadc.pitch = 0;
        quadc.yawa   = 0;
        quadc.yawv   = 0;
    quadc.yaw   = 0;


    quadc.alt = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [quadc] = control_algorithm(quadc)

quadc.direction = quadc.alt - quadc.z;
%% Begin lift-off
if quadc.z == 0.01;
    quadc.pwm1 = quadc.pwm1+1;
    quadc.pwm2 = quadc.pwm2+1;
    quadc.pwm3 = quadc.pwm3+1;
    quadc.pwm4 = quadc.pwm4+1;
    quadc = on_ground(quadc);
end

if quadc.za > 0 && quadc.hover == 0;
    quadc.hover = quadc.pwm1;
end

if quadc.direction > 0
    [quadc.pwm1,quadc.pwm2,quadc.pwm3,quadc.pwm4] = go_up(quadc);
else
    [quadc.pwm1,quadc.pwm2,quadc.pwm3,quadc.pwm4] = go_down(quadc);
end

end


function [pwm1, pwm2, pwm3, pwm4] = go_up(quadc)
pwm1 = quadc.pwm1;    
pwm2 = quadc.pwm2;
pwm3 = quadc.pwm3;
pwm4 = quadc.pwm4;
%% Up Reach Altitude
    if quadc.z > quadc.alt && quadc.zv > 0;
        pwm1 = 127+quadc.left;
        pwm2 = 127+quadc.front;
        pwm3 = 127+quadc.right;
        pwm4 = 127+quadc.back;
    end

    
    if quadc.zv < 0
       pwm1 = quadc.hover-1+quadc.left; 
       pwm2 = quadc.hover-1+quadc.front; 
       pwm3 = quadc.hover-1+quadc.right; 
       pwm4 = quadc.hover-1+quadc.back; 
       
       if quadc.z < quadc.alt
            pwm1 = 250+quadc.left;
            pwm2 = 250+quadc.front;
            pwm3 = 250+quadc.right;
            pwm4 = 250+quadc.back;
        end
    end
end
function [pwm1, pwm2, pwm3, pwm4] = go_down(quadc)
pwm1 = quadc.pwm1;    
pwm2 = quadc.pwm2;
pwm3 = quadc.pwm3;
pwm4 = quadc.pwm4;   
%% Down Reach Altitude
    if quadc.z < quadc.alt && quadc.zv < 0;
        pwm1 = 250+quadc.left;
        pwm2 = 250+quadc.back;
        pwm3 = 250+quadc.right;
        pwm4 = 250+quadc.front;
    end

    if quadc.zv > 0
       pwm1 = quadc.hover+1+quadc.left;
       pwm2 = quadc.hover+1+quadc.front;
       pwm3 = quadc.hover+1+quadc.right;
       pwm4 = quadc.hover+1+quadc.back;
       
        if quadc.z > quadc.alt
            pwm1 = quadc.hover-10+quadc.left;
            pwm2 = quadc.hover-10+quadc.front;
            pwm3 = quadc.hover-10+quadc.right;
            pwm4 = quadc.hover-10+quadc.back;
        end
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