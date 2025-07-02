function dX = formation_control_displacement(t, X)
global L
global Desired_P

% show the simulation running time
t

dX = -kron(L,eye(2))*(X-Desired_P);
end