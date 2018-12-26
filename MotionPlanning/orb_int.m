function dY = orb_int(T,Y,Cube,Switch,Sun,m0,mu_s,I_inv,rho,V,F,E,F_tilde,E_tilde,G)


%% Velocities
Y1 = [Y(5); Y(6); Y(7); Y(8)];

%% Accelerations
rho=Cube.rho;
V = Cube.Polyhedron.Vertices;
F = Cube.Polyhedron.Facets;
E = Cube.Polyhedron.Edges;
F_tilde = Cube.Polyhedron.F_tilde;
E_tilde = Cube.Polyhedron.E_tilde;
G = 6.674080000000000e-11; 
[g_A,g_I, U, Wf,W] = F_G(Y(1:3,1),Y(17:20,1),rho,V,F,E,F_tilde,E_tilde,G)
[F_3BP_I,F_3BP_B,a_3BP_I] = F_3BP(Y(1:3,1),Sun,mu_s,Y(13:16,1),Switch,m0);
[F_SRP_B,F_SRP_T,F_SRP_I,a_SRP_T] = F_SRP(SC,Sun,Y(13:16,1),Switch,m0);

Y2 = [(g_I(1)+a_3BP_I(1)+a_SRP_T(1)); (g_I(2)+a_3BP_I(2)+a_SRP_T(2)); (g_I(3)+a_3BP_I(3)+a_SRP_T(3)); 0];

%% Torques
T_SRP = cross(Y(1:3,:),F_SRP_B)';
T_3BP = cross(Y(1:3,:),F_3BP_B)';
T_GG = cross (Y(1:3,:),F_GG_B)';

T_D = T_SRP + T_3BP + T_GG;

%% Attitude dynamics
w_BI_dot = I_inv * (cross(w_BI, I*w_BI) + T_D + T_C);

Y3 = [w_BI_dot; 0];

%% Attitude kinematics
W = [0 w_BI(3) -w_BI(2) w_BI(1)
     -w_BI(3) 0 w_BI(1) w_BI(2)
     w_BI(2) -w_BI(1) 0 w_BI(3)
     -w_BI(1) -w_BI(2) -w_BI(3) 0];
 
q_BI_dot = 0.5*W*q_BI;

Y4 = q_BI_dot;

%% Asteroid dynamics
w_AI = Cube.w_AI;
Wa = [0 w_AI(3) -w_AI(2) w_AI(1)
     -w_AI(3) 0 w_AI(1) w_AI(2)
     w_AI(2) -w_AI(1) 0 w_AI(3)
     -w_AI(1) -w_AI(2) -w_AI(3) 0];
 
q_AI_dot = 0.5*Wa*q_AI;

Y5 = q_AI_dot;

%% State
dY = [Y1; Y2; Y3; Y4; Y5];


end