% function[] = ini_models_DQ()
%%-----Initialize for Acikmese test case------%%
% Asteroid descent problem for ECOS with Successive Convexification
% Shriya Hazra, 31-Jul-2018 
clear all
clc
global CONSTANTS PARAMS Switch ITR Kleopatra; 

%% Successive Convexification
%if gravity is constant
CONSTANTS.g = [-1; 0; 0];
CONSTANTS.G = 6.67408e-11;
CONSTANTS.w_AI =[0;0;0;0;0;0;0;0];

CONSTANTS.alpha0 = 0.1;

m0 = 2;
mf = 0.75; 
CONSTANTS.J = 0.5*eye(3);

CONSTANTS.F1 = 0.5;
CONSTANTS.F2 = 3;
CONSTANTS.r_F = [-1;0;0];

dq0 = Q2DQ([0;0;0;1],[2;1;0],2);
dw0 = [0;0;0;0;-1; 0.2; 0;0]; 
dF0 = [0; 0; 2;0;cross(CONSTANTS.r_F,[0,0,2])';0]; 
dF_dot0 = [0; 0; 0; 0; 0; 0; 0; 0];

dqf = Q2DQ([0;0;0;1],[0;0;0],2);
dwf = [0;0;0;0;-0.1; 0; 0;0]; 
dFf = [0; 0; 0.75; 0;cross(CONSTANTS.r_F,[0,0,0.75])';0]; 
dF_dotf = [0; 0; 0; 0; 0; 0; 0; 0];

CONSTANTS.x0 = [m0; dq0; dw0; dF0; dF_dot0];  %state bounds
CONSTANTS.xf = [mf; dqf; dwf; dFf; dF_dotf];
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
CONSTANTS.Alpha = 2;
CONSTANTS.Beta = 3.2;
CONSTANTS.i_max = 10;

%penalty weights
CONSTANTS.w_vc = 1e2;
CONSTANTS.w_tr = 0.1;
Switch.virtual_control_on = 1;
Switch.trust_region_on = 1;

%linear constraints control
Switch.constant_grav_on = 1;
Switch.discrete_higherorder_on = 0;
Switch.mass_lower_boundary_on = 1;
Switch.thrust_upper_boundary_on = 1;
Switch.thrust_lower_boundary_on = 1;

%conic constraints control
Switch.ang_rate_on = 1;
Switch.glideslope_on = 0;
Switch.tilt_ang_on = 0;
Switch.gimbal_ang_on = 0;

%initialize iter 1 state solution
PARAMS.n_state = length(CONSTANTS.x0);
PARAMS.n_control = length(dF_dotf);
PARAMS.n_virt = length(CONSTANTS.x0);
PARAMS.n_slack = 1;
PARAMS.n_tr = 1;

[Kleopatra] = Get_Asteroid('Kleopatra',1);
first_sol_DQ;
%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                end