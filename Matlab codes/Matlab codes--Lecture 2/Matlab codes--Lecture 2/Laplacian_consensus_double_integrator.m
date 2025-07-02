function dX = Laplacian_consensus_double_integrator(t,X)

% ODE solver for the linear differential system 
% [t,Y] = ode45(@Laplacian_consensus_double_integrator,[0:0.001:10],3*(rand(10,1)-0.5));



% After the simulation from the ODE solver, plot the trajectory
% using plot(t, Y)

% show the simulation runing time
t

p = X(1:5);
v = X(6:10);

% the gain to the velocity consensus part
k_v = 2; 

dX = zeros(10,1);

% the Laplacian matrix
L = [1	-1	0	0	0;
    -1	3  -1  0   -1;
    0   -1  3   -1  -1;
    0   0  -1   2  -1;
    0  -1  -1  -1   3];

% consensus dynamics
dX(1:5) = v;
dX(6:10) = -L*p - k_v*L*v;

% when finish runing the ODE solver, run the following to plot the
% trajectories
% plot(t, Y(:,1:5)); % plot the position trajectory (position consensus-->flocking)
% plot(t, Y(:,6:10))   % plot the velocity trajectory (velocity consensus)
    