function tau = heading_control(eta_hat, nu_hat, psi_d, r_d, u_d, ud_dot, c)
% Controller for LOS guidance during an evasive maneuver.
    %% Unpack input
    psi = eta_hat(3);
    r = nu_hat(3);
    
    %% Surge force (speed controller)
%     tau_X = u_d*c.Xu/(c.t_thr-1); %(6.136)
%     tau_X = ((c.m - c.Xuu)*ud_dot - c.Xu*u_d) / (1-c.t_thr);
    [M,C,D] = comp_matrices_surge_decoupled_mod(nu_hat);
    nu_d = [u_d 0 0]';
    nu_d_dot = [ud_dot 0 0]';
    tau_d = M*nu_d_dot + (C+D)*nu_d;
    tau_X = tau_d(1); % use only desired surge force.
    if tau_X < 0
        tau_X = 0;
    end

    %% Yaw force (heading control)
    psi_e = ssa(psi - psi_d);
    r_e = r - r_d;
    tau_N = -c.Kp * psi_e - c.Kd * r_e; % PD control law. Eq (15.160) in Fossen
    
    %% Total force vector
    tau = [tau_X, 0, tau_N];
    tau = thrust_limitation(tau,1); % limit thrust
end