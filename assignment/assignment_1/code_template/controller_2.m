%

close all;

A=[0.9987 0 0.0009; 0 0.9988 0.0011; 0.0009 0.0011 0.9975];

%
B=[0.6524 0 -0.9989 0.0003; 0 0.7076 0.0006 -0.7063; 0.0003 0.0004 0.9983 0.7055];

% steady state
xs=[450; 400; 350];

% steady-state input values
us=[0.9627; 0.7556; 0.2166; 0.2074];

desired_poles = [-0.2, -0.2, -0.2];
K = place(A, B, desired_poles);

% LQR controller
q=[2,2,2];
Q=diag(q);
r=[0.1,0.1,1,1];
R=diag(r);
[K_LQR, ~, ~] = dlqr(A, B, Q, R);


% Simulation parameters
T = 10;     % Total simulation time (seconds)
dt = 0.1;   % Time step (seconds)
num_steps = T / dt;

% Initial state
x0 = [0; 0; 0];

% Preallocate arrays to store results
time = zeros(1, num_steps);
heights = zeros(3, num_steps);

% Apply LQR control in a loop
x = x0;
xi(:,1) = x-xs;
for k = 1:num_steps
    % Apply LQR control
    mu = -K_LQR * xi(:,k);

    % Update system state using the dynamics equation
    % x = A * (x-xs) + B * ue + xs;
    xi(:,k+1) = A * xi(:,k)  + B * mu;

    % Store results
    time(k) = k * dt;
    heights(:, k) = xi(:,k)+xs;
end

% Plot the results
figure;
plot(time, heights(1, :), 'r', 'LineWidth', 2, 'DisplayName', 'Tank 1');
hold on;
plot(time, heights(2, :), 'g', 'LineWidth', 2, 'DisplayName', 'Tank 2');
plot(time, heights(3, :), 'b', 'LineWidth', 2, 'DisplayName', 'Tank 3');
xlabel('Time (s)');
ylabel('Height');
legend('Location', 'Best');
title('Pole placement Control of Three-Tank System');
grid on;

T= A-B*K;