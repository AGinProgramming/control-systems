
clear
% run one of the following lines:
% The two functions perform the same
% [t,Y] = ode45(@formation_control_distance_rectangular,[0:0.01:1],[0 0  -1 4  5 3  3 0]);
[t,Y] = ode45(@formation_control_distance_rectangular2,[0:0.01:1],[0 0  -1 4  5 3  3 0]);



a1 = Y(:,1); b1 = Y(:,2);
a2 = Y(:,3); b2 = Y(:,4); 
a3 = Y(:,5); b3 = Y(:,6); 
a4 = Y(:,7); b4 = Y(:,8); 

dis12 = 3; dis23 = 4; dis34 = 3; dis14 = 4; 
dis13 = 5;  

e12 = (a1-a2).^2 + (b1-b2).^2 - dis12^2;
e23 = (a2-a3).^2 + (b2-b3).^2 - dis23^2;
e34 = (a3-a4).^2 + (b3-b4).^2 - dis34^2;
e14 = (a1-a4).^2 + (b1-b4).^2 - dis14^2;
e13 = (a1-a3).^2 + (b1-b3).^2 - dis13^2;
% t = [0:0.001:2.5];
e_matrix = [e12; e23 ; e34; e14; e13];
for i = 1:size(t)
    e_norm(i) = norm(e_matrix(i,:));
end

% Uncomment the following lines to plot the V function
% V = (e12+e23+e34+e14+e13);
% figure
% plot(t,V,'r-.','LineWidth',1.5)
% legend('V')
% figure
% plot(t,V,'k-.',t,e12,'-.r',t,e23,'--g',t,e34,':r',t,e14,'k:',t,e13,'b-',t,e24,'-.b','LineWidth',2)

plot(t,e12,'-.r',t,e23,'--g',t,e34,':r',t,e14,'b:',t,e13,'b-','LineWidth',2)
legend('e_{12}','e_{23}','e_{34}','e_{14}','e_{13}')
xlabel('t')
ylabel('distance error')
% axis([0 2 -10 15])

figure
plot(Y(1,1),Y(1,2),'ro','MarkerFaceColor','r')
hold on
plot(Y(1,3),Y(1,4),'go','MarkerFaceColor','g')
plot(Y(1,5),Y(1,6),'bo','MarkerFaceColor','b')
plot(Y(1,7),Y(1,8),'ko','MarkerFaceColor','k')
plot(Y(:,1),Y(:,2),'-.r',Y(:,3),Y(:,4),'--g',Y(:,5),Y(:,6),'b:',Y(:,7),Y(:,8),'k:','LineWidth',2.5)
legend('Agent 1','Agent 2','Agent 3','Agent 4')
% hold on
% plot(Y(:,3),Y(:,4),'g--','LineWidth',2)
% hold on
% plot(Y(:,5),Y(:,6),'b:','LineWidth',2)
plot(Y(end,1),Y(end,2),'rs','MarkerFaceColor','r')
hold on
plot(Y(end,3),Y(end,4),'gs','MarkerFaceColor','g')
plot(Y(end,5),Y(end,6),'bs','MarkerFaceColor','b')
plot(Y(end,7),Y(end,8),'ks','MarkerFaceColor','k')

% Centroid position
Centroid_x = (Y(:,1) + Y(:,3) + Y(:,5)+Y(:,7))/4;
Centroid_y = (Y(:,2) + Y(:,4) + Y(:,6)+ Y(:,8))/4;


hold on 

plot(Y(:,1),Y(:,2),'-.r',Y(:,3),Y(:,4),'--g',Y(:,5),Y(:,6),'b:',Y(:,7),Y(:,8),'k:','LineWidth',2.5)
legend('Agent 1','Agent 2','Agent 3','Agent 4')
hold on 
plot(Centroid_x(end), Centroid_y(end),'kh','MarkerFaceColor','k','MarkerSize',10)
% legend('Formation centroid')

% Xpoint = [Y(end,1),Y(end,3),Y(end,5),Y(end,1)];
% Ypoint = [Y(end,2),Y(end,4),Y(end,6),Y(end,2)];
    Xpoint = [Y(end,1),Y(end,3),Y(end,5),Y(end,7),Y(end,1),Y(end,5), Y(end,3),  Y(end,7)];
    Ypoint = [Y(end,2),Y(end,4),Y(end,6),Y(end,8),Y(end,2),Y(end,6), Y(end,4),  Y(end,8)];
    
Xpoint_initial = [Y(1,1),Y(1,3),Y(1,5),Y(1,7),Y(1,1),Y(1,5), Y(1,3), Y(1,7)];
Ypoint_initial = [Y(1,2),Y(1,4),Y(1,6),Y(1,8),Y(1,2),Y(1,6), Y(1,4), Y(1,8)];


hold on
plot(Xpoint,Ypoint,'r','LineWidth',2);
plot(Xpoint_initial,Ypoint_initial,'b:','LineWidth',1.2);
axis equal
axis([-1.5 5.5 -1.5 5.5])




