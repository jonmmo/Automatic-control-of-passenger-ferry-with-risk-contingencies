for i=2:iteration
    %% Supervisor function
    [mode, WP] = supervisor(eta_dock(1:2), eta_0(1:2), mode, x(4:6,i-1), x(1:3, i-1), COLAV_index, alpha_d, p_o, indicator);
    if i == 540
%         p_o = [28 30 27 23;
%                27 33 35 37];
%        p_o = [31 ; 38];
%        p_o = [34; 34]; 
%         p_o = [30  40; 
%                44  38];
%           p_o = [27; 40];
    end
     %% Change parameters for simulation
    [mode, p_o] = change_params(sim, mode, p_o, i);
    
    %% Guidance
    if mode == 1        % undocking
        WP = WP(:,1:2); % undocking waypoints 
        [eta_d, v, pd_s, pd_ss, COLAV_index, alpha_d] = guidance_CBF(WP, h, p_o, 1, x(4:6, i-1), x(1:3,i-1));
    elseif mode == 2    % crossing
        WP = WP(:,2:3); % crossing waypoints
        [eta_d, v, pd_s, pd_ss, COLAV_index, alpha_d] = guidance_CBF(WP, h, p_o, 2, x(4:6, i-1), x(1:3,i-1));
    elseif mode == 3    % docking
        if isempty(dWP)
            dWP = docking_wps(x(4:6,i-1), eta_dock, p);
        end
        [~, ~, d1_pos, d2_pos, d1, d2] = distance_sensors(2.4, 2.4, 0.7, -0.7, eta_dock, x(4:6,i-1)); 
        [eta_d, v, pd_s, pd_ss] = dock_guidance(dWP, h, d1, d2, d_const);
    elseif mode == 5   % evasive maneuver starboard
        [psi_d, r_d, u_d, ud_dot] = guidance_turning_circle(x(4:6, i-1), x(1:3,i-1), h, p_o(:, COLAV_index), 1);
    elseif mode == 6   % evasive maneuver port 
        [psi_d, r_d, u_d, ud_dot] = guidance_turning_circle(x(4:6, i-1), x(1:3,i-1), h, p_o(:, COLAV_index), -1);
    elseif mode == 7
        if DP_flag == 0
            eta_d = x(4:6, i-1); % set desired DP position to position when DP-mode is entered
            DP_flag = 1;
        end
    end
    
    %% Control 
    if mode == 4 % crash stop
        if cs_flag == 0
            psi_d = x(6, i-1) + atan(x(2,i-1)/ x(1,i-1)); %atan2(x(2,i-1), x(1,i-1)); % set desired heading to course angle when entering crash stop
            cs_flag = 1;
            start_time = i;
        end
        x(7:9,i-1) = crash_stop_control(x(1:3,i-1), x(4:6, i-1), psi_d, hc_const);
    elseif mode == 5 || mode == 6 % evasive maneuver
        x(7:9,i-1) = heading_control(x(4:6,i-1), x(1:3,i-1), psi_d, r_d, u_d, ud_dot, hc_const);
    elseif mode == 7 % DP
        x(7:9,i-1) = backstepping(x(4:6,i-1), x(1:3,i-1), eta_d, [0, 0]', zeros(2,2), zeros(2,2), c_const);
    else % nominal control 
        x(7:9,i-1) = backstepping(x(4:6,i-1), x(1:3,i-1), eta_d, v, pd_s, pd_ss, c_const);
    end
        
    %% Vessel dynamics
    [t, x_out] = ode45(@milliAmpere_vessel_dynamics_surge_decopuled_no_thrusters,[0,h],x(1:9,i-1));
    x(1:9,i) = x_out(end,:)';

    %% Write to desired position
    if mode == 5 || mode == 6 % evasive manuever - only desired heading
        x(12, i) = psi_d;
        ud(i) = u_d;
    elseif mode == 4          % crash stop - p_d = p_d_last
        x(10:11, i) = x(10:11, i-1);
        x(12, i) = psi_d;
    else                      % nominal control and DP. 
        x(10:12, i) = eta_d; 
%         if norm(eta_d(1:2) - x(4:5, i)) > d_safe  % too far from p_d, enter MRC
%             COLAV_index = -1;
%         end
    end

end
