function dX = Van_der_Pol_oscillator_synchronization(t,X)  

% [t,Y] = ode45(@Van_der_Pol_oscillator_synchronization,[0:0.001:4],[5*(rand(50,1)-0.5).', 10*(rand(50,1)-0.5).', rand(50,1).'*2].');

%Global parameters for graphs
global G;

% the coupling gain
global k;

% Construct the Laplacian matrix
L =  diag(sum(G)) - G;

% show the simulation time
t

% system parameters for the Van_der_Pol_oscillator model
w =1;
a =1;
b =1;

mu = sin(t);
 

% Suppose 100 nodes
x = X(1:100);
y = X(101:200);


dX = zeros(200,1);


dX(1:100) = w*y - a/3*x.^3 - b*x - k*L*x;
dX(101:200) = -w*x+mu/w   - k*L*y;




