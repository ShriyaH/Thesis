% function[] = Initialize_models
%%-----Initialize Models------%%

% Change switch values to 0/1 for enabling respective parameters
% Set Integrator initial and final times as required
% Build asteroid model using required key names (Kleopatra,Kleopatra,Itokawa,Churyumov). For Cuboid Asteroid,input dimensions xyz.
% Build SC model using required key names (Rosetta,Osiris)

global CONSTANTS Switch T Sun SC Samples Constraints Kleopatra

%% Astronomical Constants
CONSTANTS.G = 6.67408e-11;       %gravitational constant
CONSTANTS.mass_s = 1.988*10^30;  %mass of sun, kg
CONSTANTS.mu_s = CONSTANTS.G*CONSTANTS.mass_s;       %standard grav parameter of the sun
CONSTANTS.c = 3e8;               %vel of light, m/s
CONSTANTS.AU = 1.49e11;          %1AU, m
CONSTANTS.flux = 1361;           %sun flux 1AU, W/m2
CONSTANTS.ge = 9.807;            %earth grav acc

%% Dynamics controls
Switch.point_mass = 0;
Switch.poly_grav = 0;
Switch.SRP = 1;
Switch.TBP = 0;
Switch.GG = 0;
Switch.Ast_rot = 1;
Switch.Control = 0;
Switch.Mapping =0;
Switch.Descent = 0;

%% SC Model
[SC] = Get_SC( 'Rosetta' );                  %Get the SC polyhedron model and its properties

%% Asteroid Model
% [Cube] = Get_Cube(60e3,30e3,40e3,G);       %Get the simple cube polyhedron and properties 
[Kleopatra] = Get_Asteroid('Kleopatra',1);   %Get the asteroid polyhedron and properties
Kleopatra.n_lm = 20;                                %number of landmarks
[Kleopatra.lm_coord,Kleopatra.lm_n,Kleopatra.lm_r] = Get_landmarks(Kleopatra,Kleopatra.n_lm);     %Get the landmarks on the asteroid surface
% Ast_model(Kleopatra);

%% Sun Model
[Sun] = Get_sun(Kleopatra,CONSTANTS.mu_s);              %Get the orbit of asteroid, sun position in the inertial frame 

%% Illumination Model
Constraints.Imp_Ellipse = MinVolEllipse(Kleopatra.Polyhedron.Vertices', 0.001); %Minimum volume impact ellipsoid 
Constraints.Esc_Vert = Kleopatra.Polyhedron.Vertices + sign(Kleopatra.Polyhedron.Vertices)*100e3;  %vertices of escape volume

%% Integrator Initial values
% T.t0 = 0;
% T.tf = 2*86400;      % Propagate for two days
% T.nsteps = T.tf/60;  % Integration per minute

%% Motion Planning 
if Switch.Mapping
    Samples.n_v = 50;           %number of samples delta v
    Samples.r = 15e3;           %max altitude
    Samples.dv_max = [15 15 15];    %max allowable delta v
    Samples.dt_max = 60;            %max allowable delta t relative to initial epoch

    [Samples] = Samples_init(Samples.n_v,Samples.r,Samples.dv_max,0.05,Kleopatra);  %Velocity space and domain space
end

%% Successive Convexification
if Switch.Descent
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

    first_sol_DQ;
end
% end