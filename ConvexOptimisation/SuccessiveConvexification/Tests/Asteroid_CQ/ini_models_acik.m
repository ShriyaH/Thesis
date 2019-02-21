function[] = ini_models_acik
%%-----Initialize for Acikmese test case------%%
% Asteroid descent problem for ECOS with Successive Convexification
% Shriya Hazra, 31-Jul-2018 
clear all
clc
global CONSTANTS PARAMS Switch ITR; 

%% Successive Convexification
CONSTANTS.g = [-0.07; 0; 0]';
CONSTANTS.alpha0 = 0.0005;
CONSTANTS.m0 = 2000; %mass bounds
CONSTANTS.mf = 750;
CONSTANTS.J = 3*CONSTANTS.m0/12.*eye(3); %5 .* eye(3) 

CONSTANTS.T1 = 30;
CONSTANTS.T2 = 300;

CONSTANTS.r_T = [-1;0;0];

CONSTANTS.x0 = [CONSTANTS.m0; 2000; 1000; 0; -10; 0; 0; 0; 0; 0; 1; 0; 0; 0; 140; 0; 0; 0; 0; 0];  %state bounds
CONSTANTS.xf = [CONSTANTS.mf; 0; 0; 0; -0.1; 0; 0; 0; 0; 0; 1; 0; 0; 0; 52.5; 0; 0; 0; 0; 0];

CONSTANTS.t0 = 0;  %initial time
CONSTANTS.tf = 20;  %closed time
CONSTANTS.nodes = 30;

CONSTANTS.w_max = deg2rad(20);
CONSTANTS.theta_gs = deg2rad(20);
CONSTANTS.theta_tilt = deg2rad(20);
CONSTANTS.theta_gm = deg2rad(10);

%trust region cost change ratio constraints
CONSTANTS.rho0 = 0;
CONSTANTS.rho1 = 0.25;
CONSTANTS.rho2 = 0.9;
% CONSTANTS.Alpha = 1.02; %15 max itr
CONSTANTS.Alpha = 1;  %10 max itr
% CONSTANTS.Alpha = 1.2;  %10 max iteration also for 15 %also for no angular constraints
CONSTANTS.Beta = 2;
CONSTANTS.i_max = 10;
CONSTANTS.tol = 0;

%penalty weights
% CONSTANTS.w_vc = 136; %no angular constraints
% CONSTANTS.w_vc = 160; %10/15 max itr
%329,1,1.05: 5 optimal,ang consts off
CONSTANTS.w_vc = 10000; %10/15 max itr %%%for all angular constraints and acik plots
CONSTANTS.w_tr = 1; %%%for all angular constraints and acik plots 
Switch.virtual_control_on = 1;
Switch.trust_region_on = 1;

%linear constraints control
Switch.discrete_higherorder_on = 0;
Switch.mass_lower_boundary_on = 1;
Switch.mass_upper_boundary_on = 1;
Switch.thrust_upper_boundary_on = 1;
Switch.thrust_lower_boundary_on = 1;

%conic constraints control
Switch.ang_rate_on = 1;
Switch.glideslope_on = 1;
Switch.tilt_ang_on = 1;
Switch.gimbal_ang_on = 1;

%initialize iter 1 state solution
PARAMS.n_state = length(CONSTANTS.x0);
PARAMS.n_control = 3;
PARAMS.n_virt = length(CONSTANTS.x0);
PARAMS.n_slack = 1;
PARAMS.n_tr = 1;

first_sol_acik;
end