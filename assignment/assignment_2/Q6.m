% 
A=[0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B=[0 0; 0 0; 1 0; 0 1];
C=[1 0 0 0; 0 1 0 0];

% define waypoints
s1 = [20; 10; 0; 0];
s2 = [50; 10; 0; 0];
s3 = [75; 10; 0; 0];
s4 = [80; 30; 0; 0];
s5 = [80; 50; 0; 0];
s6 = [20; 40; 0; 0];
s7 = [50; 60; 0; 0];

% Updated LQR controller
Q = diag([1, 1, 1, 1]);
R = eye(2);
[K_LQR, ~, ~] = lqr(A, B, Q, R);

% Define waypoints
waypoints=[s1 s2 s3 s4 s5 s6 s7];
current_waypoint = 1; % Index of the current waypoint
update_interval = 20; % Time interval to update the waypoint (adjust as needed)

% Initial condition at s0
s0=[0; 0; 2; 1];

initial_square_home = [10 0 20 0 10 10 20 10 0 0 0 0 0 0];
Desired_P_square_water = [70 40 80 40 70 50 80 50 0 0 0 0 0 0];
Desired_P_ternary_bush = [70 0 80 0 75 10 75 5*sqrt(3)/3 0 0 0 0 0 0];


% Time span for simulation
tspan = 0:1:120;

for t = 0:1:120
    % Function handle for the system dynamics with LQR control and reference signal
    odefun1 = @(t, x) quadrotor_ode(t, x, A, B, K_LQR, waypoints);
    
    % Solve the system using ode45
    [t, x] = ode15s(odefun1, tspan, s0);
end
    
%     % Extract the output (position) from the state vector
%     y = C * x';
for t = 0:1:80
    odefun2 = @(t,Y) Triangle_flocking_double_displacement(t, Desired_P_square_water);

    [t,Y] = ode15s(odefun2, tspan, initial_square_home);

end

for t = 80:1:120

    odefun3 = @(t,Z) Triangle_flocking_double_displacement_ternary(t, Desired_P_ternary_bush);

    [t,Z] = ode15s(odefun3, tspan, Desired_P_square_water);

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
    hold on;
    plot(Y(:,1),Y(:,2),'-.r',Y(:,3),Y(:,4),'--g',Y(:,5),Y(:,6),'b:',Y(:,7),Y(:,8),'k:','LineWidth',2.5)
    legend('Agent 1','Agent 2','Agent 3','Leader agent')
    hold on;
    % Plot lines connecting drones at the last timestamp
    last_idx = length(t);
    plot([Y(last_idx, 7), Y(last_idx, 1)], [Y(last_idx, 8), Y(last_idx, 2)], 'c-', 'LineWidth', 1.5);
    plot([Y(last_idx, 1), Y(last_idx, 5)], [Y(last_idx, 2), Y(last_idx, 6)], 'c-', 'LineWidth', 1.5);
    plot([Y(last_idx, 5), Y(last_idx, 3)], [Y(last_idx, 6), Y(last_idx, 4)], 'c-', 'LineWidth', 1.5);
    plot([Y(last_idx, 3), Y(last_idx, 7)], [Y(last_idx, 4), Y(last_idx, 8)], 'c-', 'LineWidth', 1.5);
    hold on;
    plot(Z(:,1),Z(:,2),'-.r',Z(:,3),Z(:,4),'--g',Z(:,5),Z(:,6),'b:',Z(:,7),Z(:,8),'k:','LineWidth',2.5)
    legend('Agent 1','Agent 2','Agent 3','Leader agent')
    hold on;
    % Plot lines connecting drones at the last timestamp
    n_agents = 3;  % Number of agents
    for i = 1:(n_agents + 1)
        for j = 1:(n_agents + 1)
            if i ~= j
                plot([Z(last_idx, (j-1)*2+1), Z(last_idx, (i-1)*2+1)], [Z(last_idx, (j-1)*2+2), Z(last_idx, (i-1)*2+2)], 'c-', 'LineWidth', 1.5);
            end
        end
    end
    
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

function dX = Triangle_flocking_double_displacement_ternary(t,X)


% show the simulation running time
t

leader_x = 2;
leader_y = 1;

% the desired relative position vector: p_ij_star
p_l1_star = [0, 10].';  % desired relative position between agent 1 and the leader;
p_12_star = [10, 0].';
p_13_star = [5, 5*sqrt(3)].';  % Adjusted for square formation
p_23_star = [5, 5*sqrt(3)/3].';  % Adjusted for square formation

p_21_star = -p_12_star;
p_31_star = -p_13_star;
p_32_star = -p_23_star;


% the followers' position
a1 = X(1); b1 = X(2);
a2 = X(3); b2 = X(4); 
a3 = X(5); b3 = X(6); 

% the leader's position
a4 = X(7); b4 = X(8); 

% the follower velocity
va1 = X(9); vb1 = X(10);
va2 = X(11); vb2 = X(12); 
va3 = X(13); vb3 = X(14); 

% the leader
% va4 = X(15); vb4 = X(16);

dX = zeros(14,1);

dX(1) = va1;
dX(2) = vb1;

dX(3) = va2;
dX(4) = vb2;

dX(5) = va3;
dX(6) = vb3;

dX(7) = leader_x;
dX(8) = leader_y;

% the agent 1(a1,b1): the adjacent follower to the leader
dX(9) = va2-va1 + va3-va1 + leader_x-va1 + (a2-a1)-p_21_star(1) + (a3-a1)-p_31_star(1) + (a4-a1)-p_l1_star(1);
dX(10) = vb2-vb1 + vb3-vb1 + leader_y-vb1 + (b2-b1)-p_21_star(2) + (b3-b1)-p_31_star(2) + (b4-b1)-p_l1_star(2);

% the agent 2(a2,b2)
dX(11) = va1 -va2 + va3 -va2 + (a1-a2)-p_12_star(1) + (a3-a2)-p_32_star(1);
dX(12) = vb1 -vb2 + vb3 -vb2 + (b1-b2)-p_12_star(2) + (b3-b2)-p_32_star(2);

% the agent 3(a3,b3)
dX(13) = va1-va3 + va2-va3 + (a1-a3)-p_13_star(1) + (a2-a3)-p_23_star(1);
dX(14) = vb1-vb3 + vb2-vb3 + (b1-b3)-p_13_star(2) + (b2-b3)-p_23_star(2);

end

function dX = Triangle_flocking_double_displacement(t,X)


% show the simulation running time
t

leader_x = 2;
leader_y = 1;

% the desired relative position vector: p_ij_star
p_l1_star = [0, 10].';  % desired relative position between agent 1 and the leader;
p_12_star = [10, 0].';
p_13_star = [10, 10].';  % Adjusted for square formation
p_23_star = [0, 10].';  % Adjusted for square formation

p_21_star = -p_12_star;
p_31_star = -p_13_star;
p_32_star = -p_23_star;


% the followers' position
a1 = X(1); b1 = X(2);
a2 = X(3); b2 = X(4); 
a3 = X(5); b3 = X(6); 

% the leader's position
a4 = X(7); b4 = X(8); 

% the follower velocity
va1 = X(9); vb1 = X(10);
va2 = X(11); vb2 = X(12); 
va3 = X(13); vb3 = X(14); 

% the leader
% va4 = X(15); vb4 = X(16);

dX = zeros(14,1);

dX(1) = va1;
dX(2) = vb1;

dX(3) = va2;
dX(4) = vb2;

dX(5) = va3;
dX(6) = vb3;

dX(7) = leader_x;
dX(8) = leader_y;

% the agent 1(a1,b1): the adjacent follower to the leader
dX(9) = va2-va1 + va3-va1 + leader_x-va1 + (a2-a1)-p_21_star(1) + (a3-a1)-p_31_star(1) + (a4-a1)-p_l1_star(1);
dX(10) = vb2-vb1 + vb3-vb1 + leader_y-vb1 + (b2-b1)-p_21_star(2) + (b3-b1)-p_31_star(2) + (b4-b1)-p_l1_star(2);

% the agent 2(a2,b2)
dX(11) = va1 -va2 + va3 -va2 + (a1-a2)-p_12_star(1) + (a3-a2)-p_32_star(1);
dX(12) = vb1 -vb2 + vb3 -vb2 + (b1-b2)-p_12_star(2) + (b3-b2)-p_32_star(2);

% the agent 3(a3,b3)
dX(13) = va1-va3 + va2-va3 + (a1-a3)-p_13_star(1) + (a2-a3)-p_23_star(1);
dX(14) = vb1-vb3 + vb2-vb3 + (b1-b3)-p_13_star(2) + (b2-b3)-p_23_star(2);

end
