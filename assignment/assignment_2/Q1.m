% 
A=[0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B=[0 0; 0 0; 1 0; 0 1];
C=[1 0 0 0; 0 1 0 0];

% define waypoints
s1 = [10;10];
s2 = [50;10];
s3 = [80;10];
s4 = [80;30];
s5 = [80;50];

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

refSignal = @(t) [interp1([0, 20, 40, 60, 80, 100], [s1(1), s2(1), s3(1), s4(1), s5(1), s5(1)], t);
                  interp1([0, 20, 40, 60, 80, 100], [s1(2), s2(2), s3(2), s4(2), s5(2), s5(2)], t)];

% 实际求的是e，而e是x-x*，所以要把x*加上去
% Function handle for the system dynamics with LQR control
odefun = @(t, x) (A - B * K_LQR) * x;

%加if循环可以

%设计ode函数时加if在里面，使得s0不断更新，每间隔一段时间更新为上一个waypoint

%另一种更快的方法就是到达目标waypoint附近就停止，视为达到目标点，然后换下一个

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





