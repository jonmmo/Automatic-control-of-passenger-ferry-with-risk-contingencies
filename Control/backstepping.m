function tau = backstepping(eta_hat, nu_hat, eta_d, v, pd_s, pd_ss, c)
% High level control algorithm. Computes desired control force tau based on
% the current position and desired position from guidance function. 
% Based on Jensen(2020) and Skjetne(2020)
    %% Unpacking input
    pd_s1 = pd_s(:,1);
    pd_s2 = pd_s(:,2);
    pd_ss_1 = pd_ss(:,1);
    pd_ss_2 = pd_ss(:,2);
    
    %% Matrices
    R = Rzyx(0,0,eta_hat(3));
    R2 = R(1:2, 1:2);
    r = nu_hat(3);
    S2 = [0  -r;
           r   0];
    [M,~,D] = comp_matrices_surge_decoupled_mod(nu_hat);   
    
    %% Neglected states
    psi_d_dot = 0;
    psi_d_ddot = 0; 
    b_hat = [0 0 0]';
    
    %% Controller gains
    K1_p = eye(2)\c.T1p;
    k1_psi = 1/c.T1psi;
    K2 = diag([0 0 0]); 
%     K2 = M\c.T2 - D;
%     K2(3) = 0;
       
    %% Error variable 1
    p_err = eta_hat(1:2) - eta_d(1:2);
    z1_p = R2'*p_err;
    z1_psi = eta_hat(3) - eta_d(3);
    z1_psi = ssa(z1_psi);
    
    %% Speed assignments available from guidance
    v1 = v(1);
    v2 = v(2);  
    v1_s1 = 0;
    v1_s2 = 0;
    v1_dot = 0;
    v2_s1 = 0;
    v2_s2 = 0;
    v2_dot = 0;
    
    %% Virtual control
    alpha_p = -K1_p*z1_p + R2'*pd_s1*v1 + R2'*pd_s2*v2;
    alpha_psi = -k1_psi*z1_psi + psi_d_dot;
    alpha = [alpha_p' alpha_psi]';
    
    %% Error variable 2
	z2_p = nu_hat(1:2) - alpha_p;
    z2_psi = r - alpha_psi;
    z2 = [z2_p' z2_psi]';
    
    %% Update laws
    omega_1 = (c.mu1/(norm(pd_s1) + eps))*pd_s1'*R2*z1_p;
    omega_2 = (c.mu2/(norm(pd_s2) + eps))*pd_s2'*R2*z1_p;

    %% Speed assignments with update laws
    s1_dot = v1 + omega_1;
    s2_dot = v2 + omega_2;
    
    %% Virtual control derivative
    alpha_p_dot = K1_p*S2*z1_p - K1_p*nu_hat(1:2) + ...
    K1_p*R2'*(pd_s1*s1_dot + pd_s2*s2_dot) - ... 
    S2*R2'*(pd_s1*v1 + pd_s2*v2) + ....
    R2'*(pd_ss_1*v1*s1_dot + pd_s1*v1_s1*s1_dot + ....
    pd_s1*v1_s2*s2_dot + pd_s1*v1_dot) + ...
    R2'*(pd_ss_2*v2*s2_dot + pd_s2*v2_s1*s1_dot + ....
    pd_s2*v2_s2*s2_dot + pd_s2*v2_dot);
    alpha_psi_dot = -k1_psi*(r - psi_d_dot) + psi_d_ddot;
    alpha_dot = [alpha_p_dot' alpha_psi_dot]';
    
    tau = -K2*z2 + D*alpha - R'*b_hat + M*alpha_dot;
    
    %% Limit to feasible forces
    tau = thrust_limitation(tau, 0);
end