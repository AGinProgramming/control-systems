function dX = Laplacian_consensus(t,X)

% ODE solver for the linear differential system 
% [t,Y] = ode45(@Laplacian_consensus,[0:0.001:3],[5 -10 20  2  3]);
% initial condition [5 -10 20  2  3];


% After the simulation from the ODE solver, plot the trajectory
% using plot(t, Y)

% show the simulation runing time
t

% the Laplacian matrix
L = [1	-1	0	0	0;
    -1	3  -1  0   -1;
    0   -1  3   -1  -1;
    0   0  -1   2  -1;
    0  -1  -1  -1   3];

% consensus dynamics
dX = -2*L*X;
    