close all;

% Define the discrete-time state-space matrices
A = [-1 0; 1 -1];  % Assuming a sampling time of 1 (Ts = 1)
B = [1; 0];
C = [0 2];
D = 0;

% Define the discrete-time state-space system
sys = ss(A, B, C, D, -1);  % Sampling time = 1

% Define the time horizon for discrete simulation
T = 100;
t = 0:1:T;

% Define the input sequences for discrete simulation
u_zero = zeros(size(t));
u_sin = sin(t);

% Simulate the discrete-time system response for u(t) = 0
lsim(sys, u_zero, t,[0;0]);

% Simulate the discrete-time system response for u(t) = sin(t)
figure
lsim(sys, u_sin, t,[0;0]);

% Plot the results for discrete simulation
figure;

subplot(2, 1, 1);
stairs(t_sim, y_zero, 'b', 'LineWidth', 1.5);
hold on;
stairs(t_sim_sin, y_sin, 'r', 'LineWidth', 1.5);
title('System Response for Different Inputs (Discrete-Time)');
legend('u(t) = 0', 'u(t) = sin(t)');
xlabel('Time');
ylabel('Output (y)');

subplot(2, 1, 2);
stairs(t_sim, x_zero(:, 1), 'b', 'LineWidth', 1.5);
hold on;
stairs(t_sim_sin, x_sin(:, 1), 'r', 'LineWidth', 1.5);
title('State Variable x1 (Position) for Different Inputs (Discrete-Time)');
legend('u(t) = 0', 'u(t) = sin(t)');
xlabel('Time');
ylabel('x1');

% Display the discrete-time state-space matrices
disp('A_discrete matrix:');
disp(A);
disp('B_discrete matrix:');
disp(B);
disp('C_discrete matrix:');
disp(C);
disp('D_discrete matrix:');
disp(D);
