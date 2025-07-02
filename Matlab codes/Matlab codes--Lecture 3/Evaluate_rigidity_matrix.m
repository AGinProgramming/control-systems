% Evaluate rigidity matrix (of a triangle framework)

% The incidence matrix
H = [1	-1	0;
     0	1	-1;
     1	0  -1];
 
% matrix \bar H
I2 = diag([1 1]);
CH=kron(H,I2);

% Embedded positions (generic positions)
p1 = [2, 1]';
p2 = [3, 2]';
p3 = [1, 4]';


% % Embedded positions that reduce a triangle into a line
% % (non-generic positions that R loses rank)
% p1 = [2, 1]';
% p2 = [5, 1]';
% p3 = [8, 1]';

p = [p1',p2',p3']';

% stacked relative position vector    
z = CH*p;   

% extract relative position vector for each edge
z12 = z(1:2);
z23 = z(3:4);
z31 = z(5:6);

% Block diagonal matrix with diagonal entry of z_{ij}
Z = blkdiag(z12,z23,z31);

% the rigidity matrix
R = Z.'*CH;

% evaluate the rank to determine infinitesimal rigidity
rank(R)

