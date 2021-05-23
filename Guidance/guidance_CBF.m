function [eta_d, v, pd_s, pd_ss, obj_index, alpha_d] = guidance_CBF(WP, h, p_o, mode, eta, nu)
% guidance function to generate desired position. Based on a straight line
% parametrization with tangential heading. Safety checked by a CBF to avoid
% collisions. 
    %% Initialize
    persistent flag s_1 s_2
    
    if isempty(flag)
        s_1 = 0;
        s_2 = 0;
        flag = 1;
    end
    lambda = 0.0001;
    u_max = 0.01;
    obj_index = 0;
    alpha_d = 0;
    r_so = 10.5;
    r_si = 5;

    %% Limit nominal path parameter
    if s_1 < 0
        s_1 = 0;
    elseif s_1 >= 1
        s_1 = 1 - 1e3*eps;
    end

    %% Desired position
    p0 = WP(:,1);
    pt = WP(:,2);
    L = norm(pt-p0);
    T = (pt - p0)/L;
    S = [0 -1; 1 0];
    N = S*T;
    p_d = p0 + L*(s_1*T + s_2*N);
    
    %Derivatives
    pd_s = L*[T N];
    pd_ss = [0 0; 0 0]; % set to 0 for now
    
    %% Speed dynamics
    a_betad = 0.2;
    u_betad = 0.8;
    a_betac = 0.2;
    u_betac = 0.5;
    delta_us = 3;
    delta_s2_dot = 2;
    
    if mode == 1
        k_beta = (norm(pd_s(1))*a_betad) / u_betad^2;
        u_s = u_betad * tanh(k_beta * (s_1 + lambda));
    elseif (s_1 > (1 - lambda)/2 ) && mode == 2
        k_beta = (norm(pd_s(1))*a_betac) / u_betac^2;
        u_s = (u_betad + u_betac) * tanh(k_beta * (1 + lambda - s_1)/delta_us);
    else
        k_beta = (norm(pd_s(1))*a_betac) / u_betac^2;
        u_s = u_betad + u_betac * tanh(k_beta * (s_1 + lambda));
    end
    sigma_p = 1; %activation signal, always 1 for now
    s1_dot = sigma_p*u_s/ norm(pd_s(1));
    s2_dot = -0.25*tanh(s_2/delta_s2_dot); %bring vessel back on path, zero if on path
    
    %% Obstacle avoidance
    s_dot = [s1_dot s2_dot]';
    n_obs = size(p_o, 2); %number of obstacles
    B = zeros(n_obs,1);
    B_dot = B;
    alpha = B;
    for j = 1:n_obs %loop through obstacles, calculate B and B_dot for each
        B(j) = norm(p_d - p_o(:,j)) - r_so;
        B_dot(j) = ((p_d - p_o(:,j))' * pd_s / norm(p_d - p_o(:,j)))*s_dot;
        gamma = 1/10;
        alpha(j) = gamma*B(j);
    end

    %% If s_dot is unsafe - quadratic optimization problem to find safe s_dot
    if sum(B_dot < -alpha) ~= 0
        %weight matrix - to be tuned
        Q = [50 0; 0 1];
        %constraint matrices
        A = zeros(n_obs,2);
        b = zeros(n_obs,1);
        for k = 1:n_obs 
            A(k,:) = -(p_d - p_o(:,k))'*pd_s / norm(p_d - p_o(:,k));
            b(k,:) = alpha(k) - A(k,:)*s_dot;
        end
        %bounds
        lb = -[0.1;0.005] - s_dot;
        ub = [0.1;0.005] - s_dot;
        %find new x
        options = optimset('Display', 'off');
        [x, ~, exitflag, ~] = quadprog(Q, [0 0]', A, b,[], [],lb, ub, [0,0]', options);
        s_dot = x + s_dot;
        alpha_d = atan(s_dot(2) / s_dot(1)); % desired heading change
    end    
    
    %% Check if inside yellow area
    if ~isempty(p_o)
        for j = 1:n_obs 
            B(j) = norm(eta(1:2) - p_o(:,j)) - r_so;
        end
        if sum(B < 0)
            u = [0 0]';
            R = zeros(n_obs, 1);
            for i = 1:n_obs
                [R(i), ~] = risk_indicator(eta, nu, p_o(:,i), u, 0);
            end
            [~, obj_index] = max(R); % indicate object with biggest risk
        end
    end
        
    %% Heading
    psi_t = atan2(WP(2,2) - WP(2,1), WP(1,2) - WP(1,1)); %nominal heading
    psi_N = atan2(s_dot(2)/norm(T), s_dot(1)); %heading correction
    
    % Activation function that deactivates heading correction when close to
    % last WP
    if s_1 > 0.95
        sigma_psi = 0;
    else
        sigma_psi = 1;
    end
    psi_d = psi_t + sigma_psi*psi_N;
    
    %% Update s_1 & s_2
    s_1 = s_1 + s_dot(1)*h;
    s_2 = s_2 + s_dot(2)*h;
    v = s_dot;
    eta_d = [p_d; psi_d];
end