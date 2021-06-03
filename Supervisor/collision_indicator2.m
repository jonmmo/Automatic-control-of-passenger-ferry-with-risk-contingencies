function mode = collision_indicator2(eta, nu, p_o)
    %% Constants
    a_max = 0.42;
    omega_max = 1.2;

    %% Calculate risk for crash stop and evasive maneuver
    n_obs = size(p_o, 2);
    R_dot = zeros(3,n_obs);
    d = zeros(n_obs,1);
    for i = 1:n_obs
        p_oi = p_o(:,i);
        d(i) = norm(p_oi - eta(1:2));
        u = [-a_max, 0]';       % crash stop u
        [~, R_dot(1,i)] = risk_indicator(eta, nu, p_oi, u, 1);
        u = [0, omega_max]';     % evasive maneuver starboard
        [~, R_dot(2,i)]  = risk_indicator(eta, nu, p_oi, u, 1);
        u = [0, -omega_max]';    % evasive maneuver port
        [~, R_dot(3,i)]  = risk_indicator(eta, nu, p_oi, u, 1);
    end
    
    %% Choose mode with lowest sum of R_dot
    %R_dot = sum(R_dot, 2); does not account for distance
    R_dot_weighed = R_dot*eye(n_obs)*diag(d)^-1;
    R_dot_sum =sum(R_dot_weighed,2);
    [~, idx] = min(R_dot_sum);
    mode = idx+3;    
end