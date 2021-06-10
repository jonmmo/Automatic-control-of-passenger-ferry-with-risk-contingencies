clear all;
close all;
%runs simulation of combined maneuvering and docking operation and plots
%results


milliAmpere_setup;
% Initial states
eta_0 = [0,0,0]';
nu_0 = [0,0,0]';
tau_0 = [0,0,0]';
mode = 1;
dWP = [];

eta_dock = [80 100 5*pi/6]';
p_o = [];

% Settings
iteration = 1000;
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
indicator = 1;  % 0 means dynamics-based, 1 is RF.
sim = 2;

simulate_supervisor;

plot_results_sim2(x, WP, dWP, eta_dock, p_o)