%% define a graph
% starting nodes
s = [1 1 2 2 2 3 4 5 ];
% terminal nodes
t = [2 4 3 4 6 6 5 6 ];
% edge weights
weights = [2 1 4 5 3 3 2 2];

%% plot a graph
% label/name each node
names = {'1' '2' '3' '4' '5' '6'};
G = graph(s,t,weights,names);
plot(G,'EdgeLabel',G.Edges.Weight) % plot the graph


%% matrix representations
% Incidence matrix (convention: row-nodes and column-edges)
I = incidence(G); 
% Laplacian (unweighted)
L = laplacian(G);


W = diag([2 1 4 5 3 3 2 2]);
% Construct weighted Laplacian
L_w = I*W*I'
% eigenvalues
eig(L_w)

