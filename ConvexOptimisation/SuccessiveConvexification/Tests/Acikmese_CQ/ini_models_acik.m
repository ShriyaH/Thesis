function[] = ini_models_acik
%%-----Initialize for Acikmese test case------%%
% Asteroid descent problem for ECOS with Successive Convexification
% Shriya Hazra, 31-Jul-2018 
clear all
clc
global CONSTANTS PARAMS Switch ITR; 

%% Successive Convexification
CONSTANTS.g = [-1; 0; 0]';
CONSTANTS.alpha0 = 0.1;
CONSTANTS.m0 = 2; %mass bounds
CONSTANTS.mf = 0.75;
CONSTANTS.J = 0.5 .* eye(3);

CONSTANTS.T1 = 0.5;
CONSTANTS.T2 = 3;

CONSTANTS.r_T = [-1;0;0];

CONSTANTS.x0 = [CONSTANTS.m0; 2; 1; 0; -1; 0.2; 0; 0; 0; 0; 1; 0; 0; 0; 2; 0; 0; 0; 0; 0];  %state bounds
CONSTANTS.xf = [CONSTANTS.mf; 0; 0; 0; -0.1; 0; 0; 0; 0; 0; 1; 0; 0; 0; 0.75; 0; 0; 0; 0; 0];

CONSTANTS.t0 = 0;  %initial time
CONSTANTS.tf = 5;  %closed time
CONSTANTS.nodes = 30;

CONSTANTS.w_max = deg2rad(30);
CONSTANTS.theta_gs = deg2rad(10);
CONSTANTS.theta_tilt = deg2rad(20);
CONSTANTS.theta_gm = deg2rad(10);

%trust region cost change ratio constraints
CONSTANTS.rho0 = 0;
CONSTANTS.rho1 = 0.25;
CONSTANTS.rho2 = 0.9;
% CONSTANTS.Alpha = 1.224655;
CONSTANTS.Alpha = 1.2;
% CONSTANTS.Alpha = 1.5;
CONSTANTS.Beta = 1.2;
CONSTANTS.i_max = 10;
CONSTANTS.tol = 0;

%penalty weights
% CONSTANTS.w_vc = 145;
% CONSTANTS.w_tr = 0.05;
CONSTANTS.w_vc = 0;
CONSTANTS.w_tr = 0.05;
Switch.virtual_control_on = 1;
Switch.trust_region_on = 1;

%linear constraints control
Switch.discrete_higherorder_on = 0;
Switch.mass_lower_boundary_on = 1;
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