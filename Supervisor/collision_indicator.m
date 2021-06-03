function mode = collision_indicator(eta, nu, alpha_d, p_o)
% Indicator function to determine best action in a COLAV situation. Takes
% the vessel state and obstacle position as input, and returns best action.
% Mode 4 = crash stop  Mode 5/6 = evasive maneuver starboard/port 
    %% Input & constants
    U = sqrt(nu(1)^2 + nu(2)^2);
    r_s = 5; 
    a_max = 0.42;       % maximum acceleration
    omega_max = 1.2; %0.48;   % maximum turning rate

    %% Find distance to obstacle straight ahead
    R = Rzyx(0,0,eta(3));
    p = eta + R*[10 0 0]';
    coeff = polyfit([eta(2) p(2)], [eta(1) p(1)],1);
    [xi, yi] = linecirc(coeff(1), coeff(2), p_o(2), p_o(1), r_s);
    delta1 = norm(eta(1:2) - [yi(1) xi(1)]');
    delta2 = norm(eta(1:2) - [yi(2) xi(2)]');
    delta = min(delta1, delta2); % distance to obstacle region
    if isnan(delta)  % the course does not intersect with the obstacle
        if alpha_d > 0
            mode = 5;
        else 
            mode = 6;
        end
        return
    end
    
    %% Necessary maneuver parameters
    t = delta/U;    % time to collision
    a_safe = U /t;  % necessary acceleration
    omega_safe = alpha_d / t;   % necessary angular velocity
    
    %% Ratio between necessary and possible manauver parameters
    p_cs = abs(a_safe /a_max);
    p_em = abs(omega_safe / omega_max);
    if p_cs < p_em 
        mode = 4;
    elseif alpha_d > 0
        mode = 5;
    else
        mode = 6;
    end

end