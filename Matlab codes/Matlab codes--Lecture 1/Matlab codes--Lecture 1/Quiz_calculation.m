%% Solve the online quiz

% The adjacency matrix
A = [0 2 0 1 0 0; 2 0 4 5 0 3; 0 4 0 0 0 3; 1 5 0 0 2 0; 0 0 0 2 0 2; 0 3 3 0 2 0];

% A =[
% 
%      0     2     0     1     0     0;
%      2     0     4     5     0     3;
%      0     4     0     0     0     3;
%      1     5     0     0     2     0;
%      0     0     0     2     0     2;
%      0     3     3     0     2     0];

% The degree matrix
D = diag(sum(A));

% The Laplacian
L = D - A

% Calculate the eigenvalue
eig(L)