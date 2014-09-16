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


