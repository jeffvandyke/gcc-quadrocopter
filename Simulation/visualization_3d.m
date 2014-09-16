X = linspace(1,10,1e3);

% define quadrotor in 3D space
by90 = [cosd(90) -sind(90)   0;  sind(90)  cosd(90)   0;  0         0       1];

q1 = [  1         1    -.5
        1         1     .5
        1         0     .5
        5         0     .5
        5.5       1     .5
        6        0.5    .5
        6        -.5    .5
        5.5      -1     .5
        4.5      -1     .5
        4        -.5    .5
        4        0.5    .5
        4.5       1     .5
        5.5       1     .5
        5         0     .5
        1         0     .5
        1        -1     .5 
        1         1    -.5
        1         0    -.5
        5         0    -.5
        5.5       1    -.5
        6        0.5   -.5
        6        -.5   -.5
        5.5      -1    -.5
        4.5      -1    -.5
        4        -.5   -.5
        4        0.5   -.5
        4.5       1    -.5
        5.5       1    -.5
        5         0    -.5
        1         0    -.5
        1        -1    -.5
        1        -1     .5];

q2 = q1*by90;
q3 = q2*by90;
q4 = q3*by90;
quad = [q1; q2; q3; q4];

numSteps = length(X);

for t = 1:1:numSteps;
    
    R = [cosd(psi(t))*cosd(theta(t)) - sind(phi(t))*sind(psi(t))*sind(theta(t)), -cosd(phi(t))*sind(psi(t)), cosd(psi(t))*cosd(theta(t))+cosd(theta(t))*sind(phi(t))*sind(psi(t));
        cosd(theta(t))*sind(psi(t)) + cosd(psi(t))*sind(phi(t))*sind(theta(t)), cosd(phi(t))*cosd(psi(t)), sind(psi(t))*sind(theta(t))-cosd(psi(t))*cosd(theta(t))*sind(phi(t));
        -cosd(phi(t))*sind(theta(t)), sind(theta(t)), cosd(phi(t))*cosd(theta(t))];
    
    % graphically plot instance in 3D space:
    
    % rotate original matrix
    quad_rot = (R*quad')';
    
    % translate original matrix
    quad_final = [quad_rot(:,1)+X(t), quad_rot(:,2)+Y(t), quad_rot(:,3)+Z(t)+15];
    
    % plot new matrix in 3D space
    plot3(quad_final(:,1),quad_final(:,2),quad_final(:,3))
    
    axis([-50,50,-50,50,0,100])
    grid on
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    pause(1/30)
end
