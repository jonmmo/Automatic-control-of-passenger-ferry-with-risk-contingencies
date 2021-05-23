function plot_results_sim5(x, x1, x2, x3, WP, eta_dock, p_o)
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
plot_pos(x);
plot_pos(x1);
plot_pos(x2);
plot_pos(x3);

p = polyshape([ed ed ed-1 ed-1],[nd-2 nd+2 nd+2 nd-2]);
p = rotate(p,rad2deg(-eta_dock(3)),[ed nd]);
plot(p,'FaceColor','black','FaceAlpha',0.2);

L = 5;
B = 2.4;

for j = 1:size(p_o,2)
    viscircles(flip(p_o(:,j))', 5);
    viscircles(flip(p_o(:,j))', 10.5, 'Color', 'y');
end
plot(x(5, 530), x(4, 530), 'Bo')
plot(x1(5, 510), x1(4, 510), 'Bo')
plot(x2(5, 500), x2(4, 500), 'Bo')
plot(x3(5, 480), x3(4, 480), 'Bo')
legend('Vessel position run 1','Desired position run 1','','', ...
    'Vessel position run 2','Desired position run 2','','', ...
    'Vessel position run 3','Desired position run 3','','', ...
    'Vessel position run 4','Desired position run 4','Final position', ...
    'Initial position', 'Dock',...
    'Detection points', 'Location', 'SE');
grid on
title('Position (m)'); xlabel('East(m)'); ylabel('North(m)'); 
end

function plot_pos(x)
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