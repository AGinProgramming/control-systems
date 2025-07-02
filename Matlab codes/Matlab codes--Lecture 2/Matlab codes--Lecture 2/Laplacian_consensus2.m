function dX = Laplacian_consensus2(t,X)

% ODE solver for the linear differential system 
% [t,Y] = ode45(@Laplacian_consensus2,[0:0.001:1],[5 -10 20  2  3]);
% initial condition [5 -10 20  2  3]


% show the simulation runing time
t


% the Laplacian matrix
L = [2	-1	-1	0	0;
    -1	4  -1  -1   -1;
    -1   -1  4   -1  -1;
    0   -1  -1   3  -1;
    0  -1  -1  -1   3];

% consensus dynamics
dX = -L*X;
    