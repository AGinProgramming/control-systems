% 
A=[0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B=[0 0; 0 0; 1 0; 0 1];
C=[1 0 0 0; 0 1 0 0];

% define waypoints
s1 = [10 10];
s2 = [50 10];
s3 = [80 10];
s4 = [80 30];
s5 = [80 50];
s6 = [60 20];
s7 = [40 40];

% LQR controller
Q = diag([1, 1, 1, 1]);
R = eye(2);
[K_LQR, ~, ~] = lqr(A, B, Q, R);

% Display the LQR gain matrix
disp('LQR Gain Matrix (K_LQR):');
disp(K_LQR);

% Initial condition at s0
s0 = [20; 30; 0; 0];

% Time span for simulation
tspan = 0:0.1:100;

waypoints = [s1 s2 s3 s4 s5 s6 s7];
current_waypoint = 1; % Index of the current waypoint
update_interval = 20; % Time interval to update the waypoint (adjust as needed)



% Hybrid controller function
hybrid_controller = @(t, x) hybrid_control(t, x, A, B, K_LQR, waypoints);

% Solve the system using ode45
[t, x] = ode45(hybrid_controller, tspan, s0);

% Extract the output (position) from the state vector
y = C * x';

% Plot the results
figure;
plot(x(:, 1), x(:, 2), 'LineWidth', 2);
hold on;
plot(waypoints(:, 1), waypoints(:, 2), 'ro', 'MarkerSize', 10);
title('Quadrotor UAV Waypoint Tracking with LTL Specification');
xlabel('X Position');
ylabel('Y Position');
legend('UAV Trajectory', 'Waypoints');
grid on;

% Hybrid controller function
function dxdt = hybrid_control(t, x, A, B, K_LQR, waypoints)
    % Define LTL specification
    home_reached = norm(x(1:2) - waypoints(:, 1)) < 1;
    obstacle_reached = norm(x(1:2) - waypoints(:, 2)) < 1;
    bush_reached = norm(x(1:2) - waypoints(:, 3)) < 1;
    water_reached = norm(x(1:2) - waypoints(:, 5)) < 1;

    % LQR control law
    u = -K_LQR * (x - waypoints(:, [home_reached obstacle_reached bush_reached water_reached]));

    % System dynamics
    dxdt = A * x + B * u;
end





