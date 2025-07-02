% % 
% A=[0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
% B=[0 0; 0 0; 1 0; 0 1];
% C=[1 0 0 0; 0 1 0 0];
% 
% % define waypoints
% s1 = [10; 10; 0; 0];
% s2 = [50; 10; 0; 0];
% s3 = [80; 10; 0; 0];
% s4 = [80; 30; 0; 0];
% s5 = [80; 50; 0; 0];
% 
% % LQR controller
% Q = diag([1, 1, 1, 1]);
% R = eye(2);
% [K_LQR, ~, ~] = lqr(A, B, Q, R);
% 
% % Display the LQR gain matrix
% disp('LQR Gain Matrix (K_LQR):');
% disp(K_LQR);
% 
% % Initial condition at s0
% s0 = [0; 0; 0; 0];
% 
% % Time span for simulation
% tspan = 0:1:100;
% 
% % Define waypoints
% waypoints = [s1 s2 s3 s4 s5];
% current_waypoint = 1; % Index of the current waypoint
% update_interval = 20; % Time interval to update the waypoint (adjust as needed)
% 
% while mod(tspan, update_interval) == 0
%     if current_waypoint < size(waypoints, 2)
%             current_waypoint = current_waypoint + 1;
%             disp(['Updating current_waypoint to ' num2str(current_waypoint) ' at t = ' num2str(t)]);
%     end
% 
%     disp(current_waypoint)
% 
% 
%     % Function handle for the system dynamics with LQR control and reference signal
%     odefun = @(t, x) quadrotor_ode(t, x, A, B, K_LQR, waypoints, current_waypoint, update_interval);
% 
% 
%     % Solve the system using ode45
%     [t, x] = ode45(odefun, tspan, s0);
% 
%     % Extract the output (position) from the state vector
%     y = C * x';
% 
%     % Plot the results
%     figure;
%     plot(x(:, 1), x(:, 2), 'LineWidth', 2);
%     hold on;
%     plot([s1(1), s2(1), s3(1), s4(1), s5(1)], [s1(2), s2(2), s3(2), s4(2), s5(2)], 'ro', 'MarkerSize', 10);
%     title('Quadrotor UAV Waypoint Tracking');
%     xlabel('X Position');
%     ylabel('Y Position');
%     legend('UAV Trajectory', 'Waypoints');
%     grid on;
% 
% end
% 
% 
% function dxdt = quadrotor_ode(t, x, A, B, K_LQR, waypoints, current_waypoint, update_interval)
%     % Initialize current_waypoint on the first call
% %     if isempty(current_waypoint)
% %         current_waypoint = 1;
% %     end
% 
% %     if mod(t, update_interval) == 0
% %         if current_waypoint < size(waypoints, 2)
% %             current_waypoint = current_waypoint + 1;
% %             disp(['Updating current_waypoint to ' num2str(current_waypoint) ' at t = ' num2str(t)]);
% %         end
% %     end
% % 
% %     disp(current_waypoint)
% 
%     % Calculate reference signal based on the current waypoint
%     ref_signal = waypoints(:, current_waypoint);
% 
%     disp('ref_signal:');
%     disp(ref_signal);
% 
%     % LQR control law
%     u = -K_LQR * (x - ref_signal);
%     disp('x:');
%     disp(x);
%     disp('u:');
%     disp(u);
% 
%     % System dynamics
%     dxdt = A * x + B * u;
%     disp('dxdt:');
%     disp(dxdt);
% end




















% function dxdt = quadrotor_ode(t, x, A, B, K_LQR, waypoints, current_waypoint, update_interval)
%     %persistent current_waypoint
% 
%     % Initialize current_waypoint on the first call
%     if isempty(current_waypoint)
%         current_waypoint = 1;
%     end
%     % Update waypoint if enough time has passed
% %     disp("II: ")
% %     if mod(t, update_interval) == 0 && current_waypoint < size(waypoints, 1)
% %         disp('current way point: '+current_waypoint)
% %         current_waypoint = current_waypoint + 1;
% %     end
% 
%     %for t = 0:1:100
%     if mod(t, update_interval) == 0
%         if current_waypoint < size(waypoints, 1)
%             current_waypoint = current_waypoint + 1;
%             disp(['Updating current_waypoint to ' num2str(current_waypoint) ' at t = ' num2str(t)]);
%         end
%     end
% 
% %     if mod(t, update_interval) == 0
% %         if current_waypoint < size(waypoints, 1)
% %             current_waypoint = current_waypoint + 1;
% %             disp(['Updating current_waypoint to ' num2str(current_waypoint) ' at t = ' num2str(t)]);
% %         end
% %     end
% 
%     disp(current_waypoint)
% 
%     % Calculate reference signal based on the current waypoint
%     ref_signal = waypoints(current_waypoint, :);
% 
%     disp('ref_signal:');
%     disp(ref_signal);
% 
%     % LQR control law
%     u = -K_LQR * (x - ref_signal);
%     %u = -K_LQR * (x(1:2) - ref_signal);
% 
%     % System dynamics
%     %xdt = A * (x - ref_signal) + B * u;
% %     disp('dxdt:');
% %     disp(dxdt);
%     dxdt = A * x + B * u';
%     dxdt = dxdt(:, 1);
%     %dxdt = [dxdt(1,1); dxdt(1,2); dxdt(3,1); dxdt(3,2)];
%     disp('dxdt:');
%     disp(dxdt);
%     %end
% end


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

% LQR controller
Q = diag([1, 1, 1, 1]);
R = eye(2);
[K_LQR, ~, ~] = lqr(A, B, Q, R);

% Display the LQR gain matrix
disp('LQR Gain Matrix (K_LQR):');
disp(K_LQR);

% Initial condition at s0
s0 = [0; 0; 0; 0];

% Time span for simulation
tspan = 0:1:100;

% Define waypoints
waypoints = [s1 s2 s3 s4 s5];
current_waypoint = 1; % Index of the current waypoint
update_interval = 20; % Time interval to update the waypoint (adjust as needed)

% Function handle for the system dynamics with LQR control and reference signal
odefun = @(t, x) quadrotor_ode(t, x, A, B, K_LQR, waypoints, current_waypoint, update_interval);


% Solve the system using ode45
[t, x] = ode45(odefun, tspan, s0);

% Extract the output (position) from the state vector
y = C * x';

% Plot the results
figure;
plot(x(:, 1), x(:, 2), 'LineWidth', 2);
hold on;
plot([s1(1), s2(1), s3(1), s4(1), s5(1)], [s1(2), s2(2), s3(2), s4(2), s5(2)], 'ro', 'MarkerSize', 10);
title('Quadrotor UAV Waypoint Tracking');
xlabel('X Position');
ylabel('Y Position');
legend('UAV Trajectory', 'Waypoints');
grid on;


function dxdt = quadrotor_ode(t, x, A, B, K_LQR, waypoints, current_waypoint, update_interval)
%     % Initialize current_waypoint on the first call
%     if isempty(current_waypoint)
%         current_waypoint = 1;
%     end

    if mod(t, update_interval) == 0
        if current_waypoint < size(waypoints, 2)
            current_waypoint = current_waypoint + 1;
            disp(['Updating current_waypoint to ' num2str(current_waypoint) ' at t = ' num2str(t)]);
        end
    end

    disp(current_waypoint)

    % Calculate reference signal based on the current waypoint
    ref_signal = waypoints(:, current_waypoint);

    disp('ref_signal:');
    disp(ref_signal);

    % LQR control law
    u = -K_LQR * (x - ref_signal);
    disp('x:');
    disp(x);
    disp('u:');
    disp(u);

    % System dynamics
    dxdt = A * x + B * u;
    disp('dxdt:');
    disp(dxdt);

end
