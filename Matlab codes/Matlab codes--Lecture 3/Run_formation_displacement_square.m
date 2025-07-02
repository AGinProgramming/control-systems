clear

p0=20*rand(40,1); % initial positive vector (random)
H=eye(20); H(20,1)=-1;

% Incidence matrix for a cycle directed graph
for i=1:1:19
    H(i,i+1)=-1;
end

global Desired_P L; %Desired Positions P (Circle) and Laplacian matrix L
L=H'*H;

% construct the desired relative positions
Desired_P=[0;0];
for i=1:1:5
    R=[2*i; 0]; 
    Desired_P=[Desired_P;R];
end

for i=6:1:10
    R=[10; 2*(i-5)]; 
    Desired_P=[Desired_P;R];
end

for i=11:1:15
    R=[10-2*(i-10); 10]; 
    Desired_P=[Desired_P;R];
end

for i=16:1:19
    R=[0; 10-2*(i-15)]; 
    Desired_P=[Desired_P;R];
end


[t,p] = ode45(@formation_control_displacement,[0:150],p0);

p=p'; 

for i=1:2:39
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
plot(p(1:2:39,end), p(2:2:40,end),'r','LineWidth',2);
plot([p(39,end), p(1,end)], [p(40,end),p(2,end)],'r','LineWidth',2);

