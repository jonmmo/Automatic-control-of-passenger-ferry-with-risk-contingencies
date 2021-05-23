function tau = thrust_limitation(tau, priority)
% Function to limit the desired thrust vector to a more realistic thrust
% vector, which gives the prioritized force 80% of max. 
    %% Constants
    max_m = 1800;
    max_f = 1000;
    c_m = abs(tau(3)/max_m);
    f = sqrt(tau(1)^2 + tau(2)^2);
    c_f = abs(f/max_f);
    ratio = tau(1:2)/norm(tau(1:2));
    c_f_possible = sqrt(1-c_m^2);
    c_m_possible = sqrt(1-c_f^2);
    
    %% Determine output
    if priority == 1    % moment priority
        if c_m > 0.8
            tau(3) = 0.8*max_m*sign(tau(3));
            c_f_possible = sqrt(1 - 0.8^2);
        end
        if c_f > c_f_possible
            tau(1:2) = max_f*c_f_possible*ratio;
        end
    else                % force priority
        if c_f > 0.8
            tau(1:2) = 0.8*max_f*ratio;
            c_m_possible = sqrt(1 - 0.8^2);
        end
        if c_m > c_m_possible
            tau(3) = max_m*c_m_possible*sign(tau(3));
        end
    end
end
