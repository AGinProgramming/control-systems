clear

p0=25*rand(8,1); % initial position vector (random)
H=eye(4); H(4,1)=-1;

% Incidence matrix for cycle graph
for i=1:1:3
H(i,i+1)=-1;
end

% Desired relative positions P (Circle)
% and Laplacian matrix L
global Desired_P L; 
L=H'*H;

% construct the desired relative positions
Desired_P=[10;0];
R1 = [20;0];
R2 = [20;10];
R3 = [10;10];

Desired_P=[Desired_P;R1; R2; R3];

[t,p] = ode45(@formation_control_displacement,[0:150],p0);

p=p'; 

for i=1:2:7
    plot(p(i,:), p(i+1,:)); 
    
    hold on 
    plot(p(i,1), p(i+1,1), 'o','MarkerSize',13); 
    text_ini = text(p(i,1),p(i+1,1),num2str((i+1)/2));
    
    axis equal
    plot(p(i,end), p(i+1,end), 's','MarkerSize',13); 
    text_end = text(p(i,end),p(i+1,end),num2str((i+1)/2));
    % plot(Desired_P(i), Desired_P(i+1), 'x');
    
end

hold on
plot(p(1:2:7,end), p(2:2:8,end),'r','LineWidth',2);
plot([p(7,end), p(1,end)], [p(8,end),p(2,end)],'r','LineWidth',2);




% %% define a graph
% % starting nodes
% s = [1 1 2 2 3 3];
% % terminal nodes
% t = [2 4 3 4 1 4];
% 
% %% plot a graph
% % label/name each node
% names = {'1' '2' '3' '4'};
% G = graph(s,t,[],names);
% figure;
% plot(G);
% 
% 
% %% matrix representations
% % Incidence matrix (convention: row-nodes and column-edges)
% I = incidence(G); 
% disp('Incidence matrix:')
% disp(full(I));
% % Laplacian (unweighted)
% Laplacian = laplacian(G);
% disp('Laplacian matrix')
% disp(full(Laplacian));
% 
% % eigenvalues
% Eigen = eig(Laplacian);
% 
% disp('Eigenvalues')
% disp(Eigen);
% 
% 
% p0=20*rand(8,1); % initial positive vector (random)
% H=eye(4); H(4,1)=-1;
% 
% % Incidence matrix for a cycle directed graph
% for i=1:1:3
%     H(i,i+1)=-1;
% end
% 
% global Desired_P L; %Desired Positions P (Ternary) and Laplacian matrix L
% L=H'*H;
% 
% % construct the desired relative positions
% Desired_P=[[0;0]; [10; 0]; [5;5]; [5;10/3]];
% 
% 
% [t, p] = ode45(@formation_control_displacement, [0:150], p0);
% 
% p = p';
% 
% for i = 1:2:7
%     plot(p(i, :), p(i + 1, :));
%     hold on
%     plot(p(i, 1), p(i + 1, 1), 'o', 'MarkerSize', 13);
%     text_ini = text(p(i, 1), p(i + 1, 1), num2str((i + 1) / 2));
%     axis equal
%     plot(p(i, end), p(i + 1, end), 's', 'MarkerSize', 13);
%     text_end = text(p(i, end), p(i + 1, end), num2str((i + 1) / 2));
% end
% 
% hold on
% plot(p(1:2:7, end), p(2:2:8, end), 'r', 'LineWidth', 2);
% plot([p(7, end), p(1, end)], [p(8, end), p(2, end)], 'r', 'LineWidth', 2);
% 
% function dX = formation_control_displacement(t, X)
%     global L
%     global Desired_P
% 
%     % show the simulation running time
%     t
% 
%     % Reshape X to a column vector
%     X = reshape(X, [], 1);
% 
%     % Ensure Desired_P has the same number of rows as X
%     Desired_P = Desired_P(1:length(X));
% 
%     % Calculate the derivative
%     dX = -kron(L, eye(2)) * (X - Desired_P);
% end