function dX = formation_control_distance_rectangular2(t,X)

% ODE solver for the differential system 
% [t,Y] = ode45(@formation_control_distance_rectangular2,[0:0.01:1],[0 0  -1 4  5 3  3 0]);



% show the simulation running time
t

% the desired distances
dis12 = 3; dis23 = 4; dis34 = 3; dis14 = 4; 
dis13 = 5;  

% The incidence matrix

H = [1	-1	0	0;
     1	0	-1	0;
     1	0  0	-1;
     0	1  -1	0;
     0  0   1  -1];

 % matrix \bar H
I2 = diag([1 1]);
CH=kron(H,I2);    
    
% stacked relative position vector    
relative_position = CH*X;    

% extract relative position vector for each edge
z12 = relative_position(1:2);
z13 = relative_position(3:4);
z14 = relative_position(5:6);
z23 = relative_position(7:8);
z34 = relative_position(9:10);


% Diagonal matrix with diagonal entry of z_{ij}
Z = blkdiag(z12,z13,z14,z23,z34);

% the rigidity matrix
R = Z.'*CH;

% the squared distance errors
e12 = norm(z12)^2 - dis12^2;
e13 = norm(z13)^2 - dis13^2;
e14 = norm(z14)^2 - dis14^2;
e23 = norm(z23)^2 - dis23^2;
e34 = norm(z34)^2 - dis34^2;

% the distance error vector
e = [e12, e13, e14, e23, e34]';

% formation control dynamics
dX = -R'*e;
    