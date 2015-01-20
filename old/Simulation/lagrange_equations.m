% Lagrange Physics

syms m M g L cvv cth k K IG a zg zgdot zgdotdot z zdot zdotdot v vdot vdotdot th thdot thdotdot real

syms x1 x2 x3 x4 u real

% Kinetic Energy
xgdot = zdot+a*cos(th)*thdot;
ygdot = -a*sin(th)*thdot;
T = 0.5*m*zdot^2 + 0.5*M*(xgdot^2+ygdot^2) + 0.5*IG*thdot^2;

% Potential Energy
V = 0.5*k*(z-zg)^2 + 0.5*K*th^2 + M*g*a*cos(th);

% Nonconservative Forces
Pz = -cvv*(zdot-zgdot);
Pth = -cth*thdot;

%Equations of Motion
pTpzdot = diff(T,zdot);

ddtpTpzdot = diff(pTpzdot,z)*zdot + ...
             diff(pTpzdot,zdot)*zdotdot + ...
             diff(pTpzdot,th)*thdot + ...
             diff(pTpzdot,thdot)*thdotdot;
pTpz  = diff(T,z);
pVpz  = diff(V,z);



pTpthdot = diff(T,thdot);

ddtpTpthdot = diff(pTpthdot,z)*zdot + ...
              diff(pTpthdot,zdot)*zdotdot + ...
              diff(pTpthdot,th)*thdot + ...
              diff(pTpthdot,thdot)*thdotdot;


pTpth = diff(T,th);

pVpth = diff(V,th);


% The equation of motion for "z" (eqz = 0)
eqz = simple(ddtpTpzdot - pTpz + pVpz - Pz);

% The equation of motion for "th" (eqth = 0)
eqth = simple(ddtpTpthdot - pTpth + pVpth - Pth);

% Rewrite the EOM using relative coordinates (i.e., v = z - zg)
z = v + zg;
zdot = vdot + zgdot;
zdotdot = vdotdot + zgdotdot;

%The equation of motion for “v” (Eqv1=0)
Eqv1 = simple(subs(eqz));
%The equation of motion for “th” (Eqth1=0)
Eqth1 = simple(subs(eqth));

fprintf('\n%s\n','The EOM in relative coordinates.')
pretty(Eqv1)
pretty(Eqth1)

% Solve for vdotdot and thdotdot for the state space formulation.
Sol = solve(Eqv1,Eqth1,'vdotdot,thdotdot');
Sol.vdotdot = simple(Sol.vdotdot);
Sol.thdotdot = simple(Sol.thdotdot);

fv=subs(Sol.vdotdot,{v,vdot,th,thdot,zgdotdot},{x1,x2,x3,x4,u})

fth=subs(Sol.thdotdot,{ v,vdot,th,thdot,zgdotdot},{x1,x2,x3,x4,u})


pfvpx1=subs(diff(fv,x1),{x1,x2,x3,x4,u},{0,0,0,0,0})
pfvpx2=subs(diff(fv,x2),{x1,x2,x3,x4,u},{0,0,0,0,0})
pfvpx3=subs(diff(fv,x3),{x1,x2,x3,x4,u},{0,0,0,0,0})
pfvpx4=subs(diff(fv,x4),{x1,x2,x3,x4,u},{0,0,0,0,0})
pfvpu=subs(diff(fv,u),{x1,x2,x3,x4,u},{0,0,0,0,0})

pfthpx1=subs(diff(fth,x1),{x1,x2,x3,x4,u},{0,0,0,0,0})
pfthpx2=subs(diff(fth,x2),{x1,x2,x3,x4,u},{0,0,0,0,0})
pfthpx3=subs(diff(fth,x3),{x1,x2,x3,x4,u},{0,0,0,0,0})
pfthpx4=subs(diff(fth,x4),{x1,x2,x3,x4,u},{0,0,0,0,0})
pfthpu=subs(diff(fth,u),{x1,x2,x3,x4,u},{0,0,0,0,0})
