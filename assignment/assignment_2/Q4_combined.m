clear

% Initial positive vector (random)
p0 = 20 * rand(8, 1);

% Incidence matrix for a cycle directed graph
H = eye(4);
H(4, 1) = -1;

% Incidence matrix for a cycle directed graph
for i = 1:1:3
    H(i, i + 1) = -1;
end

global Desired_P_square Desired_P_ternary L_square L_ternary; % Desired Positions P and Laplacian matrix L

% Initial Laplacian matrix and desired positions for the square formation
L_square = H' * H;
Desired_P_square = [[0; 0]; [10; 0]; [10; 10]; [0; 10]];

% Initial Laplacian matrix and desired positions for the ternary formation
L_ternary = H' * H;
Desired_P_ternary = [[0; 0]; [10; 0]; [5; 5*sqrt(3)]; [5; 5*sqrt(3)/3]];

% Time duration for the square formation (e.g., 20 seconds)
t_square = 20;

% simulate for the square formation
[t, p] = ode45(@formation_control_displacement, [0:t_square], p0);
p = p';

for i = 1:2:7
    plot(p(i, :), p(i + 1, :));
    hold on
    plot(p(i, 1), p(i + 1, 1), 'o', 'MarkerSize', 13);
    text_ini = text(p(i, 1), p(i + 1, 1), num2str((i + 1) / 2));
    axis equal
    plot(p(i, end), p(i + 1, end), 's', 'MarkerSize', 13);
    text_end = text(p(i, end), p(i + 1, end), num2str((i + 1) / 2));
end

hold on
plot(p(1:2:7, end), p(2:2:8, end), 'r', 'LineWidth', 2);
plot([p(7, end), p(1, end)], [p(8, end), p(2, end)], 'r', 'LineWidth', 2);

% simulate for the ternary formation
[t, p] = ode45(@formation_control_displacement, [t_square:150], p(:, end));
p = p';

for i = 1:2:7
    plot(p(i, :), p(i + 1, :));
    hold on
    plot(p(i, 1), p(i + 1, 1), 'o', 'MarkerSize', 13);
    text_ini = text(p(i, 1), p(i + 1, 1), num2str((i + 1) / 2));
    axis equal
    plot(p(i, end), p(i + 1, end), 's', 'MarkerSize', 13);
    text_end = text(p(i, end), p(i + 1, end), num2str((i + 1) / 2));
end

hold on
plot(p(1:2:7, end), p(2:2:8, end), 'b', 'LineWidth', 2);
plot([p(7, end), p(1, end)], [p(8, end), p(2, end)], 'b', 'LineWidth', 2);

function dX = formation_control_displacement(t, X)
    global Desired_P_square Desired_P_ternary L_square L_ternary

    % Reshape X to a column vector
    X = reshape(X, [], 1);

    % Determine which Laplacian matrix and desired positions to use based on time
    if t <= 20
        L = L_square;
        Desired_P = Desired_P_square;
    else
        L = L_ternary;
        Desired_P = Desired_P_ternary;
    end

    % Ensure Desired_P has the same number of rows as X
    lenX = length(X);
    Desired_P = Desired_P(1:min(end, lenX));

    % Calculate the derivative
    dX = -kron(L, eye(2)) * (X - Desired_P);
end
