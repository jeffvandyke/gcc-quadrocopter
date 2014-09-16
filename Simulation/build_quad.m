function [goliath,SQR,motorColor,propColor] = build_quad()

%% build quadcopter at origin
    %wing values
    w = .050;  % width (m)
    l = .200; % length
    h = .010;  % height

 % Base is the mounting surfaces in the center of the quadrotor. (3 plates)
 %    the plates are assumed to be symmetric, and have 8 sides. Each data
 %    point for the shortest edges has a "Big" and a "Little" value
    % middle values
    xmb = .050;   % mm "X, Middle, Big"
    xml = xmb-w/sqrt(2);  %  mm   "X, Middle,Little" 
    ymb = xmb; % if the base is square
    yml = xml; % if the base is square
    zm  =  0;  % z value of the center of the base

    % top values
    xtb = .050;   %  "X,Top, Big"
    xtl = .035;   %  "X,Top, Big"
    ytb = .065;
    ytl = yml-.005;
    zt  = .020;

    % bottom values
    xbb = xmb;
    xbl = xml;
    ybb = ymb;
    ybl = yml;
    zb  = -zt;

    % motor values
    propR  = .1015;  % Radius of Propellers
    propH  = .010;     % Height of propellers
    motorR = .020;     % Radius of motors
    motorH = .040;     % height of motors
    motorColor = [0 0 1]; %RGB
    propColor  = [.5 .5 .5]; %RGB

    
    
%% Create Nx3 Matrices for each face of the Simulation's visual plot
  % Center mounting space

    top     = [-xtl    -ytb     zt
                xtl    -ytb     zt
                xtb    -ytl     zt
                xtb     ytl     zt
                xtl     ytb     zt
               -xtl     ytb     zt
               -xtb     ytl     zt
               -xtb    -ytl     zt
              ];

    middle  = [-xml    -ymb     zm
                xml    -ymb     zm
                xmb    -yml     zm
                xmb     yml     zm
                xml     ymb     zm
               -xml     ymb     zm
               -xmb     yml     zm
               -xmb    -yml     zm
              ];

    bottom  = [-xbl    -ybb     zb
                xbl    -ybb     zb
                xbb    -ybl     zb
                xbb     ybl     zb
                xbl     ybb     zb
               -xbl     ybb     zb
               -xbb     ybl     zb
               -xbb    -ybl     zb
              ];

     %% Wings   
        by90 = [cosd(90) -sind(90)   0;  %this matrix will rotate the leg by 90 degrees
                sind(90)  cosd(90)   0;  % so only one leg needs to be created.
                0         0          1];

%  naming scheme goes as follows:
%   'wing'  : part of quadrotor
%   number1 : which wing (1-4)
%   number2 : which face of wing

    wing11 = [  -xml                ymb                 zm 
                -xml-cosd(45)*l     ymb+cosd(45)*l      zm % end of wing
                -xml-cosd(45)*l     ymb+cosd(45)*l      zm-h % end of wing
                -xml-.010             ymb+.010              zm-h
                -xml                ymb                 zb
            ];
    wing12 = [  -xmb                yml                 zm %%%%%%%beginning of 2nd face
                -xmb-cosd(45)*l     yml+cosd(45)*l      zm % end of wing
                -xmb-cosd(45)*l     yml+cosd(45)*l      zm-h % end of wing
                -xmb-.010             yml+.010              zm-h
                -xmb                yml                 zb
                ];
    wing13 = [  -xmb                 yml                 zm
                -xmb-sind(45)*l      yml+cosd(45)*l      zm  % end of wing
                -xml-sind(45)*l      ymb+cosd(45)*l      zm  % end of wing
                -xml                 ymb                 zm
                ];
    wing14 = [  -xbl                 ybb                 zb
                -xbl-.010              ybb+.010              zm-h
                -xbb-.010              ybl+.010              zm-h
                -xbb                 ybl                 zb
             ];
    wing15 = [  -xbl-.010              ybb+.010              zm-h
                -xml-sind(45)*l      ymb+cosd(45)*l      zm-h  % end of wing
                -xmb-sind(45)*l      yml+cosd(45)*l      zm-h  % end of wing
                -xbb-.010              ybl+.010              zm-h
             ];

    wing21  = wing11*by90;
    wing22  = wing12*by90;
    wing23  = wing13*by90;
    wing24  = wing14*by90;
    wing25  = wing15*by90;

    wing31  = wing21*by90;
    wing32  = wing22*by90;
    wing33  = wing23*by90;
    wing34  = wing24*by90;
    wing35  = wing25*by90;

    wing41  = wing31*by90;
    wing42  = wing32*by90;
    wing43  = wing33*by90;
    wing44  = wing34*by90;
    wing45  = wing35*by90;



    %% Motors/props
     % motor:
    motor1xyz = [-(xmb+xml)/2-sind(45)*l      (yml+ymb)/2+cosd(45)*l      zm ]; % end of wing
    motor2xyz = motor1xyz*by90;
    motor3xyz = motor2xyz*by90;
    motor4xyz = motor3xyz*by90;
    
    [SQR,motor1] = cylinder_create(motor1xyz(1),motor1xyz(2),motor1xyz(3),motorR,motorH);
    [~,motor2] = cylinder_create(motor2xyz(1),motor2xyz(2),motor2xyz(3),motorR,motorH);
    [~,motor3] = cylinder_create(motor3xyz(1),motor3xyz(2),motor3xyz(3),motorR,motorH);
    [~,motor4] = cylinder_create(motor4xyz(1),motor4xyz(2),motor4xyz(3),motorR,motorH);
    
    % prop
    prop1xyz  = [-(xmb+xml)/2-sind(45)*l      (yml+ymb)/2+cosd(45)*l      zm+motorH ]; % end of wing
    prop2xyz = prop1xyz*by90;
    prop3xyz = prop2xyz*by90;
    prop4xyz = prop3xyz*by90;
    
    [~,prop1] = cylinder_create(prop1xyz(1),prop1xyz(2),prop1xyz(3),propR,propH);
    [~,prop2] = cylinder_create(prop2xyz(1),prop2xyz(2),prop2xyz(3),propR,propH);
    [~,prop3] = cylinder_create(prop3xyz(1),prop3xyz(2),prop3xyz(3),propR,propH);
    [~,prop4] = cylinder_create(prop4xyz(1),prop4xyz(2),prop4xyz(3),propR,propH);
    


%% Build GIANT matrix of all points right after eachother
%    sorry for the biblical pun, but golaith is a huge Nx3 matrix of data
%    points, and david() is a function that breaks goliath back up into the
%    individual faces of the quadrotor for plotting
goliath = [   top
            middle
            bottom
            wing11
            wing12
            wing13
            wing14
            wing15            
            wing21
            wing22
            wing23
            wing24
            wing25
            wing31
            wing32
            wing33
            wing34
            wing35
            wing41
            wing42
            wing43
            wing44
            wing45
            motor1
            motor2
            motor3
            motor4
            prop1
            prop2
            prop3
            prop4];
        
        save('goliath_xy.mat','goliath')
        
end
function [SQR, v] = cylinder_create(x,y,z,r,h)
%% Plot enclosed cylinder
ndisc = 2;
npts = 10;

% Create constant vectors
tht = linspace(0,2*pi,npts); 
zb = linspace(0,h,ndisc);

% Create cylinder
xa = repmat(r*cos(tht)+x,ndisc,1); 
ya = repmat(r*sin(tht)+y,ndisc,1);
za = repmat(zb'+z,1,npts);

% To close the ends
X = [x*ones(size(xa)); flipud(xa); x*ones(size(xa(1,:)))]; 
Y = [y*ones(size(ya)); flipud(ya); y*ones(size(ya(1,:)))];
Z = [za; flipud(za); za(1,:)];

 

% Draw cylinder
[SQR,v]= surf2patch(X,Y,Z,'triangle'); 


end