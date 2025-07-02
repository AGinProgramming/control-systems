% Define the state-space matrices
A = [-1 0; 1 -1];
B = [1; 0];
C = [0 2];  % Output is the position x
D = 0;

% Define the continuous-time state-space system
sys = ss(A, B, C, D);

% Define the time horizon
T = 100;
t = 0:1:T;

% Define the input sequences
u_zero = zeros(size(t));
u_sin = sin(t);

% Simulate the system response for u(t) = 0
[y_zero, t_sim_zero, x_zero] = lsim(sys, u_zero, t);

% Simulate the system response for u(t) = sin(t)
[y_sin, t_sim_sin, x_sin] = lsim(sys, u_sin, t);

% Plot the results
figure;

subplot(2, 1, 1);
plot(t_sim_zero, y_zero, 'b', t_sim_sin, y_sin, 'r');
title('System Response for Different Inputs');
legend('u(t) = 0', 'u(t) = sin(t)');
xlabel('Time');
ylabel('Output (y)');

subplot(2, 1, 2);
plot(t_sim_zero, x_zero(:, 1), 'b', t_sim_sin, x_sin(:, 1), 'r');
title('State Variable x1 (Position) for Different Inputs');
legend('u(t) = 0', 'u(t) = sin(t)');
xlabel('Time');
ylabel('x1');

% Display the state-space matrices
disp('A matrix:');
disp(A);
disp('B matrix:');
disp(B);
disp('C matrix:');
disp(C);
disp('D matrix:');
disp(D);
