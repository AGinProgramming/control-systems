clear all
close all


%% System dynamics
A = [-1 0; 1 -1];
B = [1;0];
C = [0 2];
D = 0;

x0 = [4,2];
Tf=100;


%% Define ...time systems
contss = ss(A,B,C,D);
discss = ss(A,B,C,D,-1);


%% Openloop controllers
t = 0:1:Tf;
u1 = zeros(1,Tf+1);
u2 = sin(t);

%% Simulations
lsim(contss,u1,t,x0);
figure
lsim(contss,u2 ,t,x0);
figure
lsim(discss,u1,t,x0);
figure
lsim(discss,u2,t,x0);

%% Simulink

out = sim('Instr1_Q3cd');