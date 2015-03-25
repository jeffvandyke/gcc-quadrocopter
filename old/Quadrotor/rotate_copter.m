function [] = rotate_copter(x,y,z,roll,pitch,yaw)
%plots copter 
% if already built, does not re-build, and plots copter where the points
% for the parts are listed in the workspace
% if not built (absense of theta), builds and plots copter at origin.


persistent  origGoliath 


if isempty(origGoliath)
    load('goliath_xy.mat');
    origGoliath = [goliath_xy, ones(length(goliath_xy),1)];
    clear goliath_xy
end

%_%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


            
%% rotate Giant matrix:
% % %     %Set up Rot matrix
% % % rolly = [  cosd(roll)       0             -sind(roll)
% % %                 0             1                 0
% % %             sind(roll)       0             cosd(roll)  ];
% % %     
% % % pitchx = [   1       0             0
% % %              0    cosd(pitch)    -sind(pitch)
% % %              0    sind(pitch)     cosd(pitch)  ];
% % %      
% % % yawz   = [  cosd(yaw)      -sind(yaw)           0
% % %             sind(yaw)       cosd(yaw)           0
% % %                 0               0               1   ];
% % %             
% % %             
% % % rot = pitchx*rolly*yawz;

% line of code below equivilant to 4 lines above (this is faster)
rot=[                              cosd(yaw)*cosd(roll),                                     -cosd(roll)*sind(yaw),   -sind(roll)
 cosd(pitch)*sind(yaw)-cosd(yaw)*sind(pitch)*sind(roll),    cosd(pitch)*cosd(yaw)+sind(pitch)*sind(roll)*sind(yaw),   -cosd(roll)*sind(pitch)
 sind(pitch)*sind(yaw)+cosd(pitch)*cosd(yaw)*sind(roll),    cosd(yaw)*sind(pitch)-cosd(pitch)*sind(roll)*sind(yaw),    cosd(pitch)*cosd(roll)  ];
        
        
A = [   rot(1,:)    x
        rot(2,:)    y
        rot(3,:)    z
        0   0   0   1   ];

new_G = A*origGoliath'; %    nx4 orig*A 4x4

goliath = new_G(1:3,:)';

plot_copter(goliath,x,y,z,roll,pitch,yaw);
end

function [] = plot_copter(goliath,x,y,z,roll,pitch,yaw)
%plots copter 
 % needs to already have been rotated. call 
%% split HUGE matrix back up

SQR = [     1     6     7;     2     7     8;     3     8     9;     4     9    10;     6    11    12;     7    12    13;     8    13    14;     9    14    15;    11    16    17;    12    17    18;    13    18    19;    14    19    20;    16    21    22;    17    22    23;    18    23    24;    19    24    25;    21    26    27;    22    27    28;    23    28    29;    24    29    30;    26    31    32;    27    32    33;    28    33    34;    29    34    35;    31    36    37;    32    37    38;    33    38    39;    34    39    40;    36    41    42;    37    42    43;    38    43    44;    39    44    45;    41    46    47;    42    47    48;    43    48    49;    44    49    50;     1     7     2;     2     8     3;    3     9     4;     4    10     5;     6    12     7;     7    13     8;     8    14     9;     9    15    10;    11    17    12;    12    18    13;    13    19    14;    14    20    15;    16    22    17;    17    23    18;    18    24    19;    19    25    20;    21    27    22;    22    28    23;    23    29    24;    24    30    25;    26    32    27;    27    33    28;    28    34    29;    29    35    30;    31    37    32;    32    38    33;    33    39    34;    34    40    35;    36    42    37;    37    43    38;    38    44    39;    39    45    40;    41    47    42;    42    48    43;    43    49    44;    44    50    45];
motorColor = [0 0 1];
propColor  = [.5 .5 .5];

[top,middle,bottom,wing11,wing12,wing13,wing14,wing15,wing21,wing22,wing23,...
    wing24,wing25,wing31,wing32,wing33,wing34,wing35,wing41,wing42,wing43,...
    wing44,wing45,motor1,motor2,motor3,motor4,prop1,prop2,prop3,prop4] ...
    = david(goliath);

%% plotting
cla

hold on
        a = subplot(30,3,1:60);
        
        
    plot3(0,0,0,'+','MarkerEdgeColor','r','MarkerSize',15)
    
    patch(top(:,1),top(:,2),top(:,3),'k');

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
    
    subplot(a)    
        axis equal

        axis([-1000,1000,-1000,1000,-1000,1000])
        view(-13.5,6)
        grid on

    xplot = subplot(30,3,70);
    yplot = subplot(30,3,71);
    zplot = subplot(30,3,72);

    rollplot = subplot(30,3,88);
    pitchplot = subplot(30,3,89);
    yawplot = subplot(30,3,90);

 % 3d solid frame
    plot(xplot,x,0,'rs','MarkerFaceColor',[1,0,0])
    plot(yplot,y,0,'rs','MarkerFaceColor',[1,0,0])
    plot(zplot,z,0,'rs','MarkerFaceColor',[1,0,0])
    
    plot(rollplot,roll,0,'rs','MarkerFaceColor',[1,0,0])
    plot(pitchplot,pitch,0,'rs','MarkerFaceColor',[1,0,0])
    plot(yawplot,yaw,0,'rs','MarkerFaceColor',[1,0,0])
   

    

    
title(a,'Quadcopter')
xlabel(a,'X')
ylabel(a,'Y')
zlabel(a,'Z')

title(xplot,'X')
title(yplot,'Y')
title(zplot,'Z')

title(rollplot,'Roll')
title(pitchplot,'Pitch')
title(yawplot,'Yaw')

axis(xplot,[-100,100,-1,1])
axis(yplot,[-100,100,-1,1])
axis(zplot,[-100,100,-1,1])

axis(rollplot,[-45,45,-1,1])
axis(pitchplot,[-45,45,-1,1])
axis(yawplot,[-180,180,-1,1])
        

end


%% knock out Goliath
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




