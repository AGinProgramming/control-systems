function dX = Laplacian_consensus_balanced(t,X)

% ODE solver for the linear differential system 
% [t,Y] = ode45(@Laplacian_consensus_balanced,[0:0.001:1],[5 -10 20  2  3]);
% initial condition [5 -10 20  2  3]


% show the simulation runing time
t


% the Laplacian matrix
L =[ 1     0     -1     0     0;
    -1     1    0     0     0;
     0     0     2     -1    -1;
     0     0    -1     2    -1;
     0    -1     0     -1     2];

% consensus dynamics
dX = -L*X;
    