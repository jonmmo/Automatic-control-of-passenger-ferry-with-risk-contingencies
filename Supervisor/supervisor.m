function [mode, WP_out] = supervisor(dock_pos, init_pos, mode, ... 
    eta, nu, COLAV_index, alpha_d, p_o, indicator)
% Supervisor function to determine operation mode
% Takes in all data available and decides on mode based on an indicator
% function.

persistent flag WP
d_undock = 10;  %distance from start dock to first WP
d_dock =15;     %distance from target dock to start docking  from crossing
d_dock_direct = 25; % distance to go directly to docking from undocking

u = nu(1);


%% Calculate transit waypoints
if isempty(flag)
    WP = zeros(2,3);
    WP(:,1) = init_pos;
    L = norm(dock_pos - init_pos);
    T = (dock_pos - init_pos)/L;
    WP(:,2) = init_pos + d_undock*T;
    WP(:,3) = dock_pos - d_dock*T;
    flag = 1;
end


%% Determine mode, different depending on current mode
% squared distance to origin and target dock
d2_init = (eta(1)-init_pos(1))^2 + (eta(2)-init_pos(2))^2; 
d2_target = (eta(1)-dock_pos(1))^2 + (eta(2)-dock_pos(2))^2; 

if mode == 1    % undocking
    if COLAV_index ~= 0
        mode = 4;   % only crash stop available in undocking phase 
    elseif d2_init > d_undock^2 && d2_target > d_dock_direct^2   
        %switch to crossing
        mode = 2;
        clear guidance_CBF
        if norm(eta(1:2) - WP(:,2)) > 0.2
            WP(:,2) = eta(1:2);
        end
    elseif d2_init > d_undock^2 && d2_target < d_dock_direct^2
        mode = 3;
    end
elseif mode == 2    % crossing
    if  COLAV_index == -1     % distance from p_d too large
        mode = 4;
    elseif COLAV_index ~= 0    % COLAV failed
        if indicator == 0
            mode = collision_indicator(eta, nu, alpha_d, ...
                p_o(:,COLAV_index));  %dynamic constraints
        else
            mode = collision_indicator2(eta, nu, p_o);          %RIF
        end
    elseif d2_target < d_dock^2
        mode = 3;
    end    
elseif mode == 3    % docking
    
elseif mode == 4    % crash stop
    if u < 0.1 
        mode = 7;
    end
elseif mode == 5 || mode == 6   % evasive maneuver
    if u < 0.1
        mode = 7;
    end
end

WP_out = WP;

end