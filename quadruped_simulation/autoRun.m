clear all
close all
clc

addpath fcns fcns_MPC
%% --- parameters ---
% ---- gait ----
% 0-trot; 1-bound; 2-pacing 3-gallop; 4-trot run; 5-crawl
% ---- andatura --
% 0-trotto; 1-vincolata dx/sx; 2-vincolata fw/bw 3-galoppo; 4-trotto corsa; 5-strisciare

for i = 0:1:5

gait = i; 
p = get_params(gait);
p.playSpeed = 1;
p.flag_movie = 1;       % 1 - make movie

dt_sim = p.simTimeStep;
SimTimeDuration = 4.5;  % [sec]
MAX_ITER = floor(SimTimeDuration/p.simTimeStep);

% desired trajectory
p.acc_d = 1;
p.vel_d = [0.5;0];
p.yaw_d = 0;

% modified parameters
p.mass = 5.5;               % 5.5
p.vel_d = [0.5;0];          % [0.5; 0]
p.mu = 1.0;                   % 1

prefix = "PLOT/";
modifica = "mu_1.5/";
cartella = modifica+"gait_"+string(gait)+"/"
prefix = char(prefix + cartella);
mkdir(prefix)

MAIN;

pause(5)
close all
clear all

end
