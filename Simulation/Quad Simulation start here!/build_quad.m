function [goliath,SQR,motorColor,propColor] = build_quad()

%% build quadcopter at origin
    %wing values
    w = 15; %width
    l = 200;%length
    h = 10; %height

    %middle values
    xmb = 50;
    xml = xmb-w/sqrt(2);
    ymb = xmb; % if the base is square
    yml = xml;
    zm  =  0;

    %top values
    xtb = 50;
    xtl = 35;
    ytb = 65;
    ytl = yml-5;
    zt  = 20;

    % bottom values
    xbb = xmb;
    xbl = xml;
    ybb = ymb;
    ybl = yml;
    zb  = -zt;

    % motor values
    propR  = 101.5;
    propH  = 10;
    motorR = 20;
    motorH = 20;
    motorColor = [0 0 1];
    propColor  = [.5 .5 .5];

    
    

    %% Center mounting space

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
        by90 = [cosd(90) -sind(90)   0;
                sind(90)  cosd(90)   0;
                0         0          1];


    wing11 = [  -xml                ymb                 zm 
                -xml-cosd(45)*l     ymb+cosd(45)*l      zm % end of wing
                -xml-cosd(45)*l     ymb+cosd(45)*l      zm-h % end of wing
                -xml-10             ymb+10              zm-h
                -xml                ymb                 zb
            ];
    wing12 = [  -xmb                yml                 zm %%%%%%%beginning of 2nd face
                -xmb-cosd(45)*l     yml+cosd(45)*l      zm % end of wing
                -xmb-cosd(45)*l     yml+cosd(45)*l      zm-h % end of wing
                -xmb-10             yml+10              zm-h
                -xmb                yml                 zb
                ];
    wing13 = [  -xmb                 yml                 zm
                -xmb-sind(45)*l      yml+cosd(45)*l      zm  % end of wing
                -xml-sind(45)*l      ymb+cosd(45)*l      zm  % end of wing
                -xml                 ymb                 zm
                ];
    wing14 = [  -xbl                 ybb                 zb
                -xbl-10              ybb+10              zm-h
                -xbb-10              ybl+10              zm-h
                -xbb                 ybl                 zb
             ];
    wing15 = [  -xbl-10              ybb+10              zm-h
                -xml-sind(45)*l      ymb+cosd(45)*l      zm-h  % end of wing
                -xmb-sind(45)*l      yml+cosd(45)*l      zm-h  % end of wing
                -xbb-10              ybl+10              zm-h
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