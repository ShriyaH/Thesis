%%-----Initialize for Acikmese test case------%%
% Asteroid descent problem for ECOS with Successive Convexification
% Shriya Hazra, 31-Jul-2018 
% clear all
clc
global CONSTANTS PARAMS Switch; 

%% Successive Convexification
%if gravity is constant
CONSTANTS.g = [-1; 0; 0];
CONSTANTS.G = 6.67408e-11;

CONSTANTS.alpha0 = 0.1;

CONSTANTS.m0 = 2;
CONSTANTS.mf = 0.75; 
CONSTANTS.J = 0.5*eye(3);

CONSTANTS.F1 = 0.5;
CONSTANTS.F2 = 3;
CONSTANTS.r_F = [-1;0;0];

CONSTANTS.dq_form = 2; %0.5*q_bi*r_i

CONSTANTS.q0 = [0;0;0;1];
CONSTANTS.qf = [0;0;0;1];
CONSTANTS.r0 = [2;1;0];  %I-frame
CONSTANTS.rf = [0;0;0];
CONSTANTS.v0 = quat_trans(CONSTANTS.q0,[-1; 0.2; 0],'n');
CONSTANTS.vf =  quat_trans(CONSTANTS.qf,[-0.1; 0; 0],'n');
CONSTANTS.w0 = [0;0;0;0];
CONSTANTS.wf = [0;0;0;0];
CONSTANTS.F0 = [2;0;0;0];
CONSTANTS.Ff = [0.75;0;0;0];

CONSTANTS.dq0 = Q2DQ(CONSTANTS.q0,CONSTANTS.r0,CONSTANTS.dq_form); 
CONSTANTS.dw0 = [CONSTANTS.w0;CONSTANTS.v0]; 
CONSTANTS.dF0 = [CONSTANTS.F0;cross(CONSTANTS.r_F,CONSTANTS.F0(1:3));0]; 
CONSTANTS.dF_dot0 = [0; 0; 0; 0; 0; 0; 0; 0];

CONSTANTS.dqf = Q2DQ(CONSTANTS.qf,CONSTANTS.rf,CONSTANTS.dq_form);
CONSTANTS.dwf = [CONSTANTS.wf;CONSTANTS.vf]; 
CONSTANTS.dFf = [CONSTANTS.Ff;cross(CONSTANTS.r_F,CONSTANTS.Ff(1:3));0]; 
CONSTANTS.dF_dotf = [0; 0; 0; 0; 0; 0; 0; 0];

CONSTANTS.x0 = [CONSTANTS.m0; CONSTANTS.dq0; CONSTANTS.dw0; CONSTANTS.dF0; CONSTANTS.dF_dot0];  %state bounds
CONSTANTS.xf = [CONSTANTS.mf; CONSTANTS.dqf; CONSTANTS.dwf; CONSTANTS.dFf; CONSTANTS.dF_dotf];
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
CONSTANTS.Alpha = 1.22;
CONSTANTS.Beta = 3;
CONSTANTS.i_max = 10;
CONSTANTS.tol = 0;

%penalty weights
CONSTANTS.w_vc = 105; %for 10 itr
CONSTANTS.w_tr = 0.5;
Switch.virtual_control_on = 1;
Switch.trust_region_on = 1;

%linear constraints control
Switch.constant_grav_on = 1;
Switch.discrete_higherorder_on = 0;
Switch.mass_lower_boundary_on = 1;
Switch.mass_upper_boundary_on = 1;
Switch.thrust_upper_boundary_on = 1;
Switch.thrust_lower_boundary_on = 1;

%conic constraints control
Switch.ang_rate_on = 1;
Switch.glideslope_on = 0;
Switch.tilt_ang_on = 1;
Switch.gimbal_ang_on = 1;

%initialize iter 1 state solution
PARAMS.n_state = length(CONSTANTS.x0);
PARAMS.n_control = length(CONSTANTS.dF_dotf);
PARAMS.n_virt = length(CONSTANTS.x0);
PARAMS.n_slack = 1;
PARAMS.n_tr = 1;

first_sol_DQ;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           