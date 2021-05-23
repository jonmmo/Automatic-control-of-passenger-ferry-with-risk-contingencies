clear all;
close all;
%runs simulation of combined maneuvering and docking operation and plots
%results


milliAmpere_setup;
% Initial states
eta_0 = [0,0,0]';
nu_0 = [0,0,0]';
tau_0 = [0,0,0]';
R = 2;
mode = 1;
dWP = [];

eta_dock = [80 100 5*pi/6]';
% WP = [0 eta_dock(1)- 15;      %NORTH 
%       0 eta_dock(2)- 15];    %EAST
nu_d = [1 1 0.1]';
p_o = [];

% Settings
iteration = 4000;
h = 0.1;

% Initialisation
x = zeros(12,iteration);    
x(1:6,1) = [nu_0;eta_0];
eta_hat = zeros(3,iteration);
docking = 0;
s2 = zeros(1,iteration);

cs_flag = 0;
DP_flag = 0;
COLAV_index = 0;
alpha_d = 0;
indicator = 0;

sim = 5;
simulate_supervisor;
clear guidance_CBF supervisor dock_guidance guidance_turning_circle;
mode = 1;
p_o = [];
x0 = x;
x = zeros(12,iteration);   
eta_0 = [5,0,0]';
x(1:6,1) = [nu_0;eta_0];
cs_flag = 0;
DP_flag = 0;
COLAV_index = 0;

sim=51;
simulate_supervisor;
clear guidance_CBF supervisor dock_guidance guidance_turning_circle;
mode = 1;
p_o = [];
x1 = x;
eta_0 = [8,0,0]';
x = zeros(12,iteration);    
x(1:6,1) = [nu_0;eta_0];
cs_flag = 0;
DP_flag = 0;
COLAV_index = 0;

sim = 52;
simulate_supervisor;
clear guidance_CBF supervisor dock_guidance guidance_turning_circle;
mode = 1;
p_o = [];
x2 = x;
eta_0 = [13,0,0]';
x = zeros(12,iteration);    
x(1:6,1) = [nu_0;eta_0];
cs_flag = 0;
DP_flag = 0;
COLAV_index = 0;

sim = 53;
simulate_supervisor;
clear guidance_CBF supervisor dock_guidance guidance_turning_circle;
x3 = x;

plot_results_sim5(x0, x1, x2, x3, WP, eta_dock, p_o);