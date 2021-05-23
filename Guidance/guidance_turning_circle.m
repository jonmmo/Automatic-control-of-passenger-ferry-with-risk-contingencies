function [psi_d, r_d, u_d_out, ud_dot_out] = guidance_turning_circle(eta_hat, nu_hat, h, p_o, q)
% Gives guidance parameters to a heading and speed controller to execute a
% 90 degrees turning circle evasive maneuver. 
    %% Unpack input
    psi = eta_hat(3);
    r = nu_hat(3);
    u = nu_hat(1);
    
    %% Initialize
    persistent flag x_d psi_init x_du
    if isempty(flag)
        flag = 1;
        x_d = [psi r 0]';
        x_du = [u 0]';
        psi_init = psi;
    end
    
    %% Heading reference model
    psi_ref = psi_init + q*pi/2;
    w_ref = 1;
    Ad = [0  1  0
          0  0  1
          -w_ref^3 -3*w_ref^2 -3*w_ref];
    Bd = [0 0 w_ref^3]';
    xd_dot = Ad*x_d + Bd*psi_ref;
    
    %% Desired speed to ramp down if outside risk zone
    R = 0;
    if ~isempty(p_o)
        [R, ~] = risk_indicator(eta_hat, nu_hat, p_o, [0, 0]', 0);
    end
    xdu_dot = [0 0]';
    if R <= 0
        w_ref = 1;
        u_ref = 0;
        A_du = [0            1
               -w_ref^2 -2*w_ref];
        B_du = [0 w_ref^2]';
        xdu_dot = A_du*x_du + B_du*u_ref;
%         x_du = [0 0];
%         xdu_dot = [0 0]';
    end
    
    %% Output
    psi_d = x_d(1);
    r_d = x_d(2);
    
    u_d_out = x_du(1);
    ud_dot_out = x_du(2);
    
    %% Integration of reference models
    x_d = x_d + xd_dot*h;
    x_du = x_du + xdu_dot*h;
end