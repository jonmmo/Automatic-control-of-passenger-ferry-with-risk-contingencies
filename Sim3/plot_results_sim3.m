function plot_results_sim3(x, x1, WP, WP1, eta_dock, p_o)
% plot results

t = linspace(0,0.1*size(x,2),size(x,2));
n = x(4,:);
e = x(5,:);
psi = rad2deg(x(6,:));
n_d = x(10,:);
e_d = x(11,:);
n_d(n_d==0) = NaN;
e_d(e_d==0) = NaN;
psi_d = rad2deg(x(12,:));
nd = eta_dock(1);
ed = eta_dock(2);

figure(1)
plot_pos(x, WP);
plot_pos(x1, WP1);


p = polyshape([ed ed ed-1 ed-1],[nd-2 nd+2 nd+2 nd-2]);
p = rotate(p,rad2deg(-eta_dock(3)),[ed nd]);
plot(p,'FaceColor','black','FaceAlpha',0.2);

for j = 1:size(p_o,2)
    viscircles(flip(p_o(:,j))', 5);
    viscircles(flip(p_o(:,j))', 10.5, 'Color', 'y');
end
plot(x(5, 530), x(4, 530), 'o')
plot(x1(5, 490), x1(4, 490), 'o')
legend('Vessel position','Desired position', ...
    'Transit waypoints', 'Final position', 'Initial position',  ...
    'Vessel position', 'Desired position', '', '', '', 'Dock',...
    'Detection point', 'Detection point', 'Location', 'SE');
grid on
title('Position (m)'); xlabel('East(m)'); ylabel('North(m)'); 
figure(2)
figure(gcf)
plot_data(x1)
figure(3)
figure(gcf)
plot_data(x)

end

function plot_pos(x, WP)
n = x(4,:);
e = x(5,:);
psi = rad2deg(x(6,:));
n_d = x(10,:);
e_d = x(11,:);
n_d(n_d==0) = NaN;
e_d(e_d==0) = NaN;

plot(e,n,'linewidth',1.5); axis('equal')
hold on
plot(e_d, n_d);
plot(WP(2,:), WP(1,:), 'go');
L = 5;
B = 2.4;
es = e(end);
ns = n(end);
p = polyshape([es+B/2 es-B/2 es-B/2 es+B/2],[ns+L/2 ns+L/2 ns-L/2 ns-L/2]);
p = rotate(p, -psi(end), [es ns]);
plot(p,'FaceColor','green','FaceAlpha',0.2);

es = e(1);
ns = n(1);
p = polyshape([es+B/2 es-B/2 es-B/2 es+B/2],[ns+L/2 ns+L/2 ns-L/2 ns-L/2]);
p = rotate(p, -psi(1), [es ns]);
plot(p,'FaceColor','blue','FaceAlpha',0.2);
end 

function plot_data(x)
t = linspace(0,0.1*size(x,2),size(x,2));
n = x(4,:);
e = x(5,:);
psi = rad2deg(x(6,:));
n_d = x(10,:);
e_d = x(11,:);
n_d(n_d==0) = NaN;
e_d(e_d==0) = NaN;
psi_d = rad2deg(x(12,:));


subplot(321)
plot(t,n,t,n_d,'linewidth',2);
title('North position(m)'); xlabel('time (s)');
subplot(322)
plot(t,e,t,e_d,'linewidth',2);
title('East position(m)'); xlabel('time (s)');
subplot(325)
plot(t,psi,t,psi_d,'linewidth',2);
title('Heading angle(deg)'); xlabel('time (s)');


subplot(323)
plot(t,x(1,:),'linewidth',2);
title('Surge velocity(m/s)'); xlabel('time (s)');
subplot(324)
plot(t,x(2,:),'linewidth',2);
title('Sway velocity(m/s)'); xlabel('time (s)');
subplot(326)
plot(t,rad2deg(x(3,:)),'linewidth',2);
title('Yaw rate(deg/s)'); xlabel('time (s)');
end