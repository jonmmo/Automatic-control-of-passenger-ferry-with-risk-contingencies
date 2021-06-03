function WP = docking_wps(eta_0, eta_dock, p)
% takes start position, dock position and ship parameters as input, and
% outputs a vector of 4 docking waypoints.

R_d = [cos(eta_dock(3)) -sin(eta_dock(3));
       sin(eta_dock(3)) cos(eta_dock(3));];
d_dist = 1.5*p.L;

p_0 = eta_0(1:2);
p_3 = eta_dock(1:2) + R_d*[0; d_dist];
p_2 = p_3 + R_d*[0; 0.5*p.L];

dist = R_d'*p_0 - R_d'*p_2; %differences in BODY-frame
p_1 = p_2 + R_d*[0.5*dist(1); 0.5*dist(2)];

WP = [p_0 p_1 p_2 p_3];
end