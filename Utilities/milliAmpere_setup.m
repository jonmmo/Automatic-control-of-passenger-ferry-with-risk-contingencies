% guidance parameters
guidance_params.u_d = 1;
guidance_params.Delta_p_1 = 0.01;
guidance_params.mu_1 = 0.01;
guidance_params.u_N = 0.1;
guidance_params.d_safe = 2.5;
guidance_params.Delta_p_2 = 3.5;
guidance_params.Delta_p_3 = 1;
guidance_params.Delta_p_4 = 2;
guidance_params.mu_2 = 0;

% max deviation from p_d
d_safe = 2;

% ship parameters
p.L = 5;

% docking parameters
d_const.d_ref = 0.2;
d_const.u_approach = 0.2;
d_const.u_dock = 0.05;
d_const.lambda =0.2;
d_const.d1_pos = 0.7;
d_const.d2_pos = -0.7;

% backstepping control parameters 
c_const.mu1 = 0;
c_const.mu2 = 0;
c_const.T1p = 10*diag([1 1]); %10*diag([1 1])
c_const.T1psi = 0.1; % 0.1
c_const.T2 = 10*diag([0.75 0.5 0.01]);

% heading control parameters
wb = 0.8; % how high can this be?
zeta = 1;
m = 5068.910;
wn = (1 / sqrt( 1 - 2*zeta^2 + sqrt( 4*zeta^4 - 4*zeta^2 + 2) )) * wb;
hc_const.Kp = m*wn^2;
hc_const.Kd = m*2*zeta*wn;
hc_const.Xu = -27.632;
hc_const.Xuu = -110.064;
hc_const.m = 2389.657020;
hc_const.t_thr = 0;