function dX = formation_control_distance_rectangular(t,X)

% [t,Y] = ode45(@formation_control_distance_rectangular,[0:0.01:100],[0 0  8 9  -7 6  -6 -8]);
% [t,Y] = ode45(@formation_control_distance_rectangular,[0:0.01:3],[0 0  -1 4  5 3  3 0]);

% For standard rectangular formation

% show the simulation time
t

% the desired distances
dis12 = 3; dis23 = 4; dis34 = 3; dis14 = 4; 
dis13 = 5;  

a1 = X(1); b1 = X(2);
a2 = X(3); b2 = X(4); 
a3 = X(5); b3 = X(6); 
a4 = X(7); b4 = X(8); 

% squared distance errors
e12 = (a1-a2)^2 + (b1-b2)^2 - dis12^2;
e23 = (a2-a3)^2 + (b2-b3)^2 - dis23^2;
e34 = (a3-a4)^2 + (b3-b4)^2 - dis34^2;
e14 = (a1-a4)^2 + (b1-b4)^2 - dis14^2;
e13 = (a1-a3)^2 + (b1-b3)^2 - dis13^2;

 

dX = zeros(8,1);

% the point 1(a1,b1)
dX(1) = -(a1-a2)*e12 - (a1-a3)*e13 - (a1-a4)*e14;
dX(2) = -(b1-b2)*e12 - (b1-b3)*e13 - (b1-b4)*e14;

% the point 2(a2,b2)
dX(3) = -(a2-a1)*(e12) - (a2-a3)*e23;
dX(4) = -(b2-b1)*(e12) - (b2-b3)*e23;

% the point 3(a3,b3)
dX(5) = -(a3-a1)*(e13) - (a3-a2)*(e23) - (a3-a4)*(e34);
dX(6) = -(b3-b1)*(e13) - (b3-b2)*(e23) - (b3-b4)*(e34);

% the point 4(a4,b4,c4)
dX(7) = -(a4-a1)*(e14) - (a4-a3)*(e34);
dX(8) = -(b4-b1)*(e14) - (b4-b3)*(e34);
