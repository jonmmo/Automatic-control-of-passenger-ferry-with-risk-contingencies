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
WP = [];

eta_dock = [80 100 5*pi/6]';
nu_d = [1 1 0.1]';
p_o = [8 ; 18];

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
sim = 6;
indicator = 1;

simulate_supervisor;
% simulate_parameter_tests;

plot_results_sim6(x, WP, dWP, eta_dock, p_o)