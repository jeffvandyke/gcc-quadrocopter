function [] = plot_copter(x,y,z,roll,pitch,yaw)
%plots copter 
% if already built, does not re-build, and plots copter where the points
% for the parts are listed in the workspace
% if not built (absense of theta), builds and plots copter at origin.


persistent first origGoliath SQR motorColor propColor

first = 1;

if first
    [origGoliath,SQR,motorColor,propColor] = build_quad();
    first = 0;
end

%_%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


            
%% rotate Giant matrix:
    %Set up Rot matrix

pitchx = [   1       0             0
            0    cosd(pitch)    -sind(pitch)
            0    sind(pitch)     cosd(pitch)  ];
     
rolly = [  cosd(roll)       0             -sind(roll)
                0             1                 0
            sind(roll)       0             cosd(roll)  ];
     
yawz   = [  cosd(yaw)      -sind(yaw)           0
            sind(yaw)       cosd(yaw)           0
                0               0               1   ];
            
            
rot = pitchx*rolly*yawz;

A = [   rot(1,:)    x
        rot(2,:)    y
        rot(3,:)    z
        0   0   0   1   ];
u = [origGoliath, ones(length(origGoliath),1)];

trans = A*u'; %   4x4 A*u nx4

goliath = trans(1:3,:)';



%% split HUGE matrix back up

[top,middle,bottom,wing11,wing12,wing13,wing14,wing15,wing21,wing22,wing23,...
    wing24,wing25,wing31,wing32,wing33,wing34,wing35,wing41,wing42,wing43,...
    wing44,wing45,motor1,motor2,motor3,motor4,prop1,prop2,prop3,prop4] ...
    = david(goliath);


%% plotting

 % 3d solid frame
    clf
    patch(top(:,1),top(:,2),top(:,3),'k')
    hold on
    patch(middle(:,1),middle(:,2),middle(:,3),'k')
    patch(bottom(:,1),bottom(:,2),bottom(:,3),'k')
    
    patch(wing11(:,1),wing11(:,2),wing11(:,3),'g')
    patch(wing12(:,1),wing12(:,2),wing12(:,3),'g')
    patch(wing13(:,1),wing13(:,2),wing13(:,3),'g')
    patch(wing14(:,1),wing14(:,2),wing14(:,3),'g')
    patch(wing15(:,1),wing15(:,2),wing15(:,3),'g')
    
    patch(wing31(:,1),wing31(:,2),wing31(:,3),'b')
    patch(wing32(:,1),wing32(:,2),wing32(:,3),'b')
    patch(wing33(:,1),wing33(:,2),wing33(:,3),'b')
    patch(wing34(:,1),wing34(:,2),wing34(:,3),'b')
    patch(wing35(:,1),wing35(:,2),wing35(:,3),'b')
    
    patch(wing21(:,1),wing21(:,2),wing21(:,3),'g')
    patch(wing22(:,1),wing22(:,2),wing22(:,3),'g')
    patch(wing23(:,1),wing23(:,2),wing23(:,3),'g')
    patch(wing24(:,1),wing24(:,2),wing24(:,3),'g')
    patch(wing25(:,1),wing25(:,2),wing25(:,3),'g')
    
    patch(wing41(:,1),wing41(:,2),wing41(:,3),'b')
    patch(wing42(:,1),wing42(:,2),wing42(:,3),'b')
    patch(wing43(:,1),wing43(:,2),wing43(:,3),'b')
    patch(wing44(:,1),wing44(:,2),wing44(:,3),'b')
    patch(wing45(:,1),wing45(:,2),wing45(:,3),'b')
    
   patch('Vertices',motor1,'Faces',SQR,'facecolor',motorColor);
   patch('Vertices',motor2,'Faces',SQR,'facecolor',motorColor);
   patch('Vertices',motor3,'Faces',SQR,'facecolor',motorColor);
   patch('Vertices',motor4,'Faces',SQR,'facecolor',motorColor);
   
   patch('Vertices',prop1,'Faces',SQR,'facecolor',propColor);
   patch('Vertices',prop2,'Faces',SQR,'facecolor',propColor);
   patch('Vertices',prop3,'Faces',SQR,'facecolor',propColor);
   patch('Vertices',prop4,'Faces',SQR,'facecolor',propColor);   
    
    xlabel('X')
    ylabel('Y')
    zlabel('Z')

    axis equal
    axis([-1000,1000,-1000,1000,-1000,1000])
    view(3)
    grid on
    
    
    

end

function [top,middle,bottom,wing11,wing12,wing13,wing14,wing15,wing21,wing22,wing23,...
    wing24,wing25,wing31,wing32,wing33,wing34,wing35,wing41,wing42,wing43,...
    wing44,wing45,motor1,motor2,motor3,motor4,prop1,prop2,prop3,prop4]...
    = david(goliath)

top         = goliath(1:8,:);
middle      = goliath(9:16,:);
bottom      = goliath(17:24,:);
wing11      = goliath(25:29,:);
wing12      = goliath(30:34,:);
wing13      = goliath(35:38,:);
wing14      = goliath(39:42,:);
wing15      = goliath(43:46,:);
wing21      = goliath(47:51,:);
wing22      = goliath(52:56,:);
wing23      = goliath(57:60,:);
wing24      = goliath(61:64,:);
wing25      = goliath(65:68,:);
wing31      = goliath(69:73,:);
wing32      = goliath(74:78,:);
wing33      = goliath(79:82,:);
wing34      = goliath(83:86,:);
wing35      = goliath(87:90,:);
wing41      = goliath(91:95,:);
wing42      = goliath(96:100,:);
wing43      = goliath(101:104,:);
wing44      = goliath(105:108,:);
wing45      = goliath(109:112,:);
motor1      = goliath(113:162,:);
motor2      = goliath(163:212,:);
motor3      = goliath(213:262,:);
motor4      = goliath(263:312,:);
prop1       = goliath(313:362,:);
prop2       = goliath(363:412,:);
prop3       = goliath(413:462,:);
prop4       = goliath(463:512,:);

end

