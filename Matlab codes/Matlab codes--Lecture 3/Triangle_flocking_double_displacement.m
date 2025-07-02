function dX = Triangle_flocking_double_displacement(t,X)

% [t,Y] = ode45(@Triangle_flocking_double_displacement,[0:0.01:100],[initial_positions, initial_velocoties]);

% show the simulation running time
t

% the leader's velocity input, to guide the whole formation
% leader_x = 2 + sin(t);
% leader_y = 2 + cos(t);

leader_x = 2;
leader_y = 1;

% the desired relative position vector: p_ij_star
p_l1_star = [2, 2].';  % desired relative position between agent 1 and the leader;
p_12_star = [4, 0].';
p_13_star = [2, 2].';
p_23_star = [-2, 2].';

p_21_star = - p_12_star;
p_31_star = - p_13_star;
p_32_star = - p_23_star;


% the followers' position
a1 = X(1); b1 = X(2);
a2 = X(3); b2 = X(4); 
a3 = X(5); b3 = X(6); 

% the leader's position
a4 = X(7); b4 = X(8); 

% the follower velocity
va1 = X(9); vb1 = X(10);
va2 = X(11); vb2 = X(12); 
va3 = X(13); vb3 = X(14); 

% the leader
% va4 = X(15); vb4 = X(16);

dX = zeros(14,1);

dX(1) = va1;
dX(2) = vb1;

dX(3) = va2;
dX(4) = vb2;

dX(5) = va3;
dX(6) = vb3;

dX(7) = leader_x;
dX(8) = leader_y;

% the following lines generate circular motion for the leader (note: the
% circle shape depends on initial positions)
% dX(7) = 0.3*b4;
% dX(8) = -0.3*a4;
% 
% leader_x = 0.3*b4;
% leader_y = -0.3*a4;

% the agent 1(a1,b1): the adjacent follower to the leader
dX(9) = va2-va1 + va3-va1 + leader_x-va1 + (a2-a1)-p_21_star(1) + (a3-a1)-p_31_star(1) + (a4-a1)-p_l1_star(1);
dX(10) = vb2-vb1 + vb3-vb1 + leader_y-vb1 + (b2-b1)-p_21_star(2) + (b3-b1)-p_31_star(2) + (b4-b1)-p_l1_star(2);

% the agent 2(a2,b2)
dX(11) = va1 -va2 + va3 -va2 + (a1-a2)-p_12_star(1) + (a3-a2)-p_32_star(1);
dX(12) = vb1 -vb2 + vb3 -vb2 + (b1-b2)-p_12_star(2) + (b3-b2)-p_32_star(2);

% the agent 3(a3,b3)
dX(13) = va1-va3 + va2-va3 + (a1-a3)-p_13_star(1) + (a2-a3)-p_23_star(1);
dX(14) = vb1-vb3 + vb2-vb3 + (b1-b3)-p_13_star(2) + (b2-b3)-p_23_star(2);

% Note: the above lines decompose the motions for each agent, and in each x/y directions. 
% One can write a more compact simulation code, using the compact
% expression of the flocking dynamics. 