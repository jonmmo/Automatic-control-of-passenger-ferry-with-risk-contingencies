function [tau, flag] = crash_stop_control( nu_hat, eta_hat, psi_d, c)
% Control function to perform crash stop. Outputs full throttle astern as
% long as the forward speed is positive. The flag represents if the vessel
% has come to a stop or not.
    %% Input & constants
    u = nu_hat(1);
    r = nu_hat(3);
    psi = eta_hat(3);
    tau = zeros(3,1);
    tau_max = 1000;
    
    %% Surge and sway force
    if u > 0 
        tau(1:2) = -tau_max*nu_hat(1:2)/norm(nu_hat(1:2)); % opposite of velocity vector
    end
    
    %% PD-controller for yaw force
    psi_e = ssa(psi - psi_d);
    r_e = r;
    tau(3) = -c.Kp * psi_e - c.Kd * r_e;    % PD control law. Eq (15.160) in Fossen
    tau = thrust_limitation(tau, 0);        % limit thrust
end