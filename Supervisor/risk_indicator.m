function [R, R_dot] = risk_indicator(eta, nu, p_o, u, region)
% Caulculates the risk function R and its derivative based on the vessel
% state, obstacle position and possible action u. The type describes which
% region the risk should be evaluated for - 0 is the yellow region and 1 is
% the red(critical) region. 
    %% Unpack input
    psi = eta(3);
    v = nu(2);
    U = sqrt(nu(1)^2 + v^2);
    
    %% Constants
    if region == 0
        r_s = 10.5;
    else
        r_s = 5;
    end
    S = [ 0 -1; 1 0];
    t_0 = 10;
    
    %% Calculate states
    chi = psi + asin(v/U);
    p = eta(1:2) - p_o;         % position relative to obstacle
    z = [cos(chi) sin(chi)]';
    
    %% Calculate R and derivative
    R = r_s - norm(p) - t_0 * (p'/norm(p)) * z;
    LgR = -(p'/norm(p)) * [z  U*S*z]; 
    LfR = - U * (p'/norm(p)) * z - t_0 * (U^2 / norm(p)) * ((p'/norm(p))* S' * z )^2;
    R_dot = LfR + LgR * u;
    
end