% Define system dynamics
A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B = [0 0; 0 0; 1 0; 0 1];

% Define LQR cost matrices
Q = diag([1, 1, 1, 1]); % Adjust the weights according to your requirements
R = eye(2);

% Solve continuous-time Algebraic Riccati Equation
[P, ~, ~] = care(A, B, Q, R);

% Calculate LQR controller gains
K_LQR = R \ B' * P;

% Display the calculated gains
disp('LQR Controller Gains (K_LQR):');
disp(K_LQR);
