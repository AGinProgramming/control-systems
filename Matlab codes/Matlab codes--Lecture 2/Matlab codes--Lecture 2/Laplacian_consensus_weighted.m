function dX = Laplacian_consensus_weighted(t,X)

% ODE solver for the linear differential system 
% [t,Y] = ode45(@Laplacian_consensus_weighted,[0:0.001:1],[5 -10 20  2  3]);
% initial condition [5 -10 20  2  3]


% show the simulation runing time
t

H =[1    -1     0     0     0;
     0     1    -1     0     0;
     0     1     0     0    -1;
     0     0     1    -1     0;
     0     0     1     0    -1;
     0     0     0     1    -1];

W = diag([2 3 2 3 1 2]);
% the Laplacian matrix
L_w = H'*W*H;

% consensus dynamics
dX = -L_w*X;
    