global G;

global k;

k = 2; 

% Generating an Erdos-Renyi graph 
% The Erdos-Renyi model is connect _n_ vertices with probability _p_.
n = 100;
p = 0.1;
G = rand(n,n) < p;
G = triu(G,1);
G = G + G';


% Run the system via ODE solver
[t,Y] = ode45(@Van_der_Pol_oscillator_synchronization,[0:0.001:1],[5*(rand(100,1)-0.5).', 10*(rand(100,1)-0.5).']);


% Plot the synchronization trajectory
figure
subplot(1,2,1)
plot(t, Y(:,1:100))
subplot(1,2,2)
plot(t, Y(:,101:200))

 