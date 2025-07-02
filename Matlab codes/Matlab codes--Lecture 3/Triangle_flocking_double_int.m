function dX = Triangle_flocking_double_int(t,X)

% [t,Y] = ode45(@Triangle_flocking_double_int,[0:0.01:100],[0 0 0 0 0 0 0 0   0 0  18 9  -17 6]);
% you may try different initial positions/velocities, to have a faster
% transient period


% show the simulation running time
t

% the leader's velocity input, to guide the whole formation
% leader_x = 2 + sin(t);
% leader_y = 2 + cos(t);

leader_x = 2;
leader_y = 1;

% the distance
dis12 = 10; dis23 = 10;  
dis13 = 10; dis14 = 10;
 
 

% the follower positiom
a1 = X(1); b1 = X(2);
a2 = X(3); b2 = X(4); 
a3 = X(5); b3 = X(6); 

% the leader
a4 = X(7); b4 = X(8); 

% the follower velocity
va1 = X(9); vb1 = X(10);
va2 = X(11); vb2 = X(12); 
va3 = X(13); vb3 = X(14); 

% the leader
% va4 = X(15); vb4 = X(16);

% distance errors
e12 = (a1-a2)^2 + (b1-b2)^2 - dis12^2;
e23 = (a2-a3)^2 + (b2-b3)^2 - dis23^2;
e13 = (a1-a3)^2 + (b1-b3)^2 - dis13^2;
e14 = (a1-a4)^2 + (b1-b4)^2 - dis14^2;


dX = zeros(14,1);

dX(1) = va1;
dX(2) = vb1;

dX(3) = va2;
dX(4) = vb2;

dX(5) = va3;
dX(6) = vb3;

dX(7) = leader_x;
dX(8) = leader_y;

% the following lines generate circular motion for the leader
% dX(7) = 0.3*b4;
% dX(8) = -0.3*a4;
% 
% leader_x = 0.3*b4;
% leader_y = -0.3*a4;

% the agent 1(a1,b1): the adjacent follower to the leader
dX(9) = va2-va1 + va3-va1 + leader_x-va1 -(a1-a2)*(e12) - (a1-a3)*e13 - (a1-a4)*e14;
dX(10) = vb2-vb1 + vb3-vb1 + leader_y-vb1 -(b1-b2)*(e12) - (b1-b3)*e13 - (b1-b4)*e14;

% the agent 2(a2,b2)
dX(11) = va1 -va2 + va3 -va2 -(a2-a1)*(e12) - (a2-a3)*e23;
dX(12) = vb1 -vb2 + vb3 -vb2 -(b2-b1)*(e12) - (b2-b3)*e23;

% the agent 3(a3,b3)
dX(13) = va1-va3 + va2-va3 -(a3-a1)*(e13) - (a3-a2)*(e23);
dX(14) = vb1-vb3 + vb2-vb3 -(b3-b1)*(e13) - (b3-b2)*(e23);