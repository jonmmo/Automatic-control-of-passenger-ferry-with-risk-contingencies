function mode = collision_indicator2(eta, nu, p_o)
    %% Constants
    a_max = 0.42;
    omega_max = 1.2; %0.48;

    %% Calculate risk for crash stop and evasive maneuver
    n_obs = size(p_o, 2);
    R_dot = zeros(3,n_obs);
    for i = 1:n_obs
        p_oi = p_o(:,i);
        u = [-a_max, 0]';       % crash stop u
        [~, R_dot(1,i)] = risk_indicator(eta, nu, p_oi, u, 1);
        u = [0, omega_max]';     % evasive maneuver starboard
        [~, R_dot(2,i)]  = risk_indicator(eta, nu, p_oi, u, 1);
        u = [0, -omega_max]';    % evasive maneuver port
        [~, R_dot(3,i)]  = risk_indicator(eta, nu, p_oi, u, 1);
    end
    
    %% Choose mode with lowest sum of R_dot
    R_dot = sum(R_dot, 2);
    [~, idx] = min(R_dot);
    mode = idx+3;    
end