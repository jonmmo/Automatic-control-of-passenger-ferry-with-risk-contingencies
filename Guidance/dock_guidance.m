function [eta_d, v, pd_s, pd_ss] = dock_guidance(WP, h, d1, d2, c) 
% ADAPTED FROM GAUSLAA (2020), CONTAINS SOME UNUSED VARIABLES/FUNCTIONS
% This function generates the desired position and heading from the current
% position of the vessel. This is done by generating a desired path at the
% first iteration, and getting the signals from that path according to
% where the vessel is. When this block detects arrival at the last WP it
% applies a heading correction to make the vessel normal to the dock, and
% starts generating a docking path along the tangential vector to the last
% segment. Basic situational awareness is implemented by checking the
% current measurement up against the previous. 
%% Initializing parameters, variables and path the first iteration
    % Declaring persistent variables
persistent flag s s2 sigma_psi sigma_p...
           psi_d_corr_save r N LinSysA Order NumSubpaths LinSysbx...
           LinSysby WPx WPy coeffa coeffb coeffa_der coeffb_der t_ph2...
           prev_d1 prev_d2 danger_flag s_dot s2_dot
if isempty(flag)
        % Initializing run variables
        t_ph2 = 0;
        s=0;
        s_dot = 0;
        flag = 1;
        s2 = 0;
        s2_dot = 0;
        sigma_psi = 0;
        sigma_p = 0;
        psi_d_corr_save = 0;
        % Initializing parameters and variables for generating path
        r = 3;
        N  = size(WP,2);
        % Generate path using GenerateHybridPath
        [NumSubpaths, Order, WPx, WPy, LinSysA, LinSysbx,...
            LinSysby, coeffa, coeffb, coeffa_der, coeffb_der]...
            = GenerateHybridPath(WP',r,c.lambda);
        % Situational awareness variables
        prev_d1 = 0;                        % Saving previous measurement
        prev_d2 = 0;                        % Saving previous measurement
        danger_flag = 0;
end

%% Checking if s is in the right, feasible interval
    if s < 0
        s = 0.0;
    elseif s >= N
        s = N-eps;
    end
    
%% Get pd and pd_der from GetHybridPathSignals
    [pd, pd_der] = GetHybridPathSignals(NumSubpaths, Order, WPx, WPy,...
    LinSysA, LinSysbx, LinSysby, coeffa, coeffb, coeffa_der, coeffb_der,s);

%% Calculating desired (x,y)_d,_d_s,_d_2s, and (psi)_d,_d_s,_d_2s
% Extracting data from pd and pd_der
    x_d=pd(1);                                   % Desired x-pos
    y_d=pd(2);                                   % Desired y-pos
    x_d_s = pd_der(1);                           % Derivative in x
    y_d_s = pd_der(2);                           % Derivative in y
    x_d_2s = pd_der(1);                          % Double derivative in x
    y_d_2s = pd_der(2);                          % Double derivative in y
    x_d_3s = pd_der(1);                          % Triple derivative in x
    y_d_3s = pd_der(2);                          % Triple derivative in y
    t_d = ([x_d_s y_d_s]'/norm([x_d_s y_d_s]')); % Tangent vector
    
    pd_s = [pd_der(:,:,1)' t_d];
    pd_ss = [pd_der(:,:,2)' [0; 0]];
% Applying heading correction if activation function == 1, otherwise
% saving correction to apply next iteration to be parallel.
    if sigma_psi == 1
        % Applying correction  to heading when arriving at WP outside dock
        psi_d_corr = psi_d_corr_save;
    else
        % Calculating and saving the needed correction for later use
        psi_d_corr_save = atan2(d1-d2,c.d1_pos-c.d2_pos);
        psi_d_corr = 0;
    end
% Caclculating desired heading and derivative. First term = path tangential 
% , second term = correction calculated above
    psi_d = atan2(y_d_s, x_d_s); % + psi_d_corr;
    psi_d_s = ((x_d_s*y_d_2s)-(x_d_2s*y_d_s))/...
               (x_d_s^2 + y_d_s^2);
    psi_d_2s =(((x_d_s*y_d_3s)-(x_d_3s*y_d_s))/...
                (x_d_s^2 + y_d_s^2))-2*(((x_d_s*y_d_2s)-(x_d_2s*y_d_s))*...
                (x_d_s*x_d_2s+y_d_s*y_d_2s))/((x_d_s^2 +y_d_s^2)^2);
            
%% Constructing outputs
% Desired eta given by two terms. First is nominal path,
% second is the docking path. 
    eta_d = [x_d, y_d, psi_d]' + s2*[t_d(1) t_d(2) 0]'; % Desired eta
    eta_d_s = [x_d_s, y_d_s, psi_d_s]';                % Der. desired eta
    eta_d_2s = [x_d_2s, y_d_2s, psi_d_2s]';            % 2xder desired eta
    s_out = s;                                      
    s=s+s_dot*h;                                       % Calculate s_1
    s2=s2+s2_dot*h;                                    % Calculate s2_dot
    s2o = s2;
    d = min([d1, d2]);                                 % Lowest sens. meas.
    N_out = N;
    q_d = [x_d,y_d];                                   % Nominal path term
    %p_dock = s2*[n_d(1) n_d(2)];                       % Docking path term
    

%% Speed assignment   
    
    v_s1 = c.u_approach/norm(eta_d_s(1:2))* tanh(((N-s))/0.1);
    v_s2 = c.u_dock*tanh((d-c.d_ref)/0.5);
    s_dot = v_s1;                          % Calculating s1_dot
    s2_dot = sigma_p*v_s2;              % Calculating s2_dot
    
    v = [s_dot, 0];
    if sigma_p 
         v = [s2_dot, 0];
    end
%% Checking if arrived at WP outside dock, setting activation function for 
% heading correction to 1 if so, applying heading correction and saving the
% time of initiation of Phase 2.
    if s>=N-0.01
        sigma_psi = 1;
        t_ph2_out = t_ph2;
    else
        t_ph2_out = nan;
        t_ph2 = t_ph2 + 0.01;
    end
    
%% Checking if heading is parallel to dock after applied heading correction
% setting activation function for s2_dot equal to 1, generating a docking
% path and making the motion controller move the ship in sway along it.
    if abs(atan2(d1-d2,c.d1_pos-c.d2_pos)) < deg2rad(5) && sigma_psi == 1
        sigma_p = 1;
    end
