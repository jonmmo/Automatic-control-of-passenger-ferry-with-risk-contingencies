function dxdt = milliAmpere_vessel_dynamics_surge_decopuled_no_thrusters(t,x)
% x is a vector of vessel states;
% x(1:3,1) are the body velocities (nu) : [surge, sway, yaw_rate]'
% x(4:6,1) are the position/pose (eta) : [north, east, heading]'
% x(7:9,1) are are the forces acting on the vessel: [X,Y,N]'
dxdt = zeros(9,1);
% nu_dynamics, dxdt(1:3,1)
[M,C,D] = comp_matrices_surge_decoupled_mod(x(1:3,1));
dxdt(1:3,1) = M\(x(7:9,1) - (C+D)*x(1:3,1));

% eta_dynamics dxdt(4:6,1);
dxdt(4:6,1) = [cos(x(6,1)),-sin(x(6,1)), 0;...
               sin(x(6,1)), cos(x(6,1)), 0;...
               0          , 0          , 1]*x(1:3,1);
end
