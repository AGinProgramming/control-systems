% 
A=[0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B=[0 0; 0 0; 1 0; 0 1];
C=[1 0 0 0; 0 1 0 0];

% define waypoints
s1 = [10; 10; 0; 0];
s2 = [50; 10; 0; 0];
s3 = [80; 10; 0; 0];
s4 = [80; 30; 0; 0];
s5 = [80; 50; 0; 0];

% Updated waypoints
s6 = [20; 40; 0; 0];  % New waypoint
s7 = [40; 60; 0; 0];  % New waypoint

% Updated LQR controller
Q = diag([1, 1, 1, 1]);
R = eye(2);
[K_LQR, ~, ~] = lqr(A, B, Q, R);

% Initial condition at s0
s0 = [0; 0; 0; 0];

% Time span for simulation
tspan = 0:1:120;

% Define waypoints
waypoints = [s1 s2 s3 s4 s5 s6 s7];
current_waypoint = 1; % Index of the current waypoint
update_interval = 20; % Time interval to update the waypoint (adjust as needed)

for t = 0:1:120
    % Function handle for the system dynamics with LQR control and reference signal
    odefun = @(t, x) quadrotor_ode(t, x, A, B, K_LQR, waypoints);
    
    
    % Solve the system using ode45
    [t, x] = ode15s(odefun, tspan, s0);
    
    % Extract the output (position) from the state vector
    y = C * x';
end
    
    % Plot the results
    figure;
    plot(x(:, 1), x(:, 2), 'LineWidth', 2);
    hold on;
    plot([s1(1), s2(1), s3(1), s4(1), s5(1), s6(1), s7(1)], [s1(2), s2(2), s3(2), s4(2), s5(2), s6(2), s7(2)], 'ro', 'MarkerSize', 10);
    title('Quadrotor UAV Waypoint Tracking');
    xlabel('X Position');
    ylabel('Y Position');
    legend('UAV Trajectory', 'Waypoints');
    
    figure;
    plot(t, x(:,1), 'r', 'LineWidth', 2);
    hold on;
    plot(t, x(:,2), '--g', 'LineWidth', 2);
    hold off;
    title('Quadrotor UAV trajectory');
    xlabel('simulation time');
    ylabel('Position');
    legend('UAV Trajectory x', 'UAV Trajectory y');
    grid on;

function dxdt = quadrotor_ode(t, x, A, B, K_LQR, waypoints)

    %waypoints = [s1 s2 s3 s4 s5];

        if t <= 20
            current_waypoint = 1;
            ref_signal = waypoints(:, current_waypoint);
            u = -K_LQR * (x - ref_signal);
            dxdt = A * x + B * u;
        elseif 20 < t && t <= 40
            current_waypoint = 6;
            ref_signal = waypoints(:, current_waypoint);
            u = -K_LQR * (x - ref_signal);
            dxdt = A * x + B * u;
        elseif 40 < t && t <= 60
            current_waypoint = 7;
            ref_signal = waypoints(:, current_waypoint);
            u = -K_LQR * (x - ref_signal);
            dxdt = A * x + B * u;
        elseif 60 < t && t <= 80
            current_waypoint = 5;
            ref_signal = waypoints(:, current_waypoint);
            u = -K_LQR * (x - ref_signal);
            dxdt = A * x + B * u;
        elseif 80 < t && t <= 100
            current_waypoint = 4;
            ref_signal = waypoints(:, current_waypoint);
            u = -K_LQR * (x - ref_signal);
            dxdt = A * x + B * u;
        elseif 100 < t && t <= 120
            current_waypoint = 3;
            ref_signal = waypoints(:, current_waypoint);
            u = -K_LQR * (x - ref_signal);
            dxdt = A * x + B * u;
        end

end




