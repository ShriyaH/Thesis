function[] = ini_models_dyn()
%%-----Initialize Models------%%
% Change switch values to 0/1 for enabling respective parameters
% Set Integrator initial and final times as required
% Build Kleopatra model using required key names (Kleopatra,Kleopatra,Itokawa,Churyumov). For Cuboid Kleopatra,input dimensions xyz.
% Build SC model using required key names (Rosetta,Osiris)

global CONSTANTS PARAMS Switch T Sun SC Samples Constraints Kleopatra

%% Astronomical Constants
CONSTANTS.G = 6.67408e-11;       %gravitational constant
CONSTANTS.mass_s = 1.988*10^30;  %mass of sun, kg
CONSTANTS.mu_s = 1.32712440018e20;       %standard grav parameter of the sun
CONSTANTS.c = 3e8;               %vel of light, m/s
CONSTANTS.AU = 1.49e11;          %1AU, m
CONSTANTS.flux = 1361;           %sun flux 1AU, W/m2
CONSTANTS.ge = 9.807;            %earth grav acc

%% Dynamics controls
Switch.constant_grav = 0;
Switch.SRP = 0;
Switch.TBP = 0;
Switch.GG = 0;
Switch.Ast_rot = 0;
Switch.Control = 0;
Switch.Mapping =0;
Switch.Descent =0;

%% SC Model
[SC] = Get_SC( 'Rosetta' );                  %Get the SC polyhedron model and its properties

%% Kleopatra Model
[Kleopatra] = Get_Asteroid('Kleopatra',1);   %Get the Kleopatra polyhedron and properties
Kleopatra.n_lm = 20;                                %number of landmarks
[Kleopatra.lm_coord,Kleopatra.lm_n,Kleopatra.lm_r] = Get_landmarks(Kleopatra,Kleopatra.n_lm);     %Get the landmarks on the Kleopatra surface

CONSTANTS.g = CONSTANTS.G*Kleopatra.mu.*[-1;0;0];
% CONSTANTS.g = [-1;0;0];
% Ast_model(Kleopatra);

%% Sun Model
[Sun] = Get_sun(Kleopatra,CONSTANTS.mu_s);              %Get the orbit of Kleopatra, sun position in the inertial frame 

%% Motion Planning 
if Switch.Mapping
    %% Integrator Initial values
    % T.t0 = 0;
    % T.tf = 2*86400;      % Propagate for two days
    % T.nsteps = T.tf/60;  % Integration per minute
    %% Illumination Model
    Constraints.Imp_Ellipse = MinVolEllipse(Kleopatra.Polyhedron.Vertices', 0.001); %Minimum volume impact ellipsoid 
    Constraints.Esc_Vert = Kleopatra.Polyhedron.Vertices + sign(Kleopatra.Polyhedron.Vertices)*100e3;  %vertices of escape volume
    Samples.n_v = 50;           %number of samples delta v
    Samples.r = 15e3;           %max altitude
    Samples.dv_max = [15 15 15];    %max allowable delta v
    Samples.dt_max = 60;            %max allowable delta t relative to initial epoch

    [Samples] = Samples_init(Samples.n_v,Samples.r,Samples.dv_max,0.05,Kleopatra);  %Velocity space and domain space
end

%% Successive Convexification
if Switch.Descent
%     CONSTANTS.alpha0 = 1/SC.v_exh;

%     CONSTANTS.m0 = SC.mass.m_i;
%     CONSTANTS.mf = SC.mass.dry; 
%     CONSTANTS.J = SC.I.I_total;
% 
%     CONSTANTS.F1 = 200;
%     CONSTANTS.F2 = 100;
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
    CONSTANTS.vf = quat_trans(CONSTANTS.qf,[-0.1; 0; 0],'n');
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
    
%     CONSTANTS.dw_AI = [Kleopatra.w_AI';0;0;0;0;0];
    CONSTANTS.dw_AI = [0;0;0;0;0;0;0;0];
    
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
    CONSTANTS.w_vc = 105.5;%for 10 itr
    % CONSTANTS.w_vc = 100;
    CONSTANTS.w_tr = 0.5;
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
    Switch.glideslope_on = 0;
    Switch.tilt_ang_on = 1;
    Switch.gimbal_ang_on = 1;

    %initialize iter 1 state solution
    PARAMS.n_state = length(CONSTANTS.x0);
    PARAMS.n_control = length(CONSTANTS.dF_dotf);
    PARAMS.n_virt = length(CONSTANTS.x0);
    PARAMS.n_slack = 1;
    PARAMS.n_tr = 1;

    first_sol(Kleopatra); 
end

end