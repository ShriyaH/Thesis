function [t,y] = pert_test(Sun,Asteroid,Switch,SC)
 %state vec x_I = [r_I v_I q_BI w_BI q_AI]  x_A = [r_A v_A q_BA w_BA q_AI]
%%--- Required inputs---%%
G = 6.674080000000000e-11; 
mu_s = 1.326807104e20;
AU = 1.496e11;
vel_light = 3e8;

%%-- Asteroid properties--%%
rho = Asteroid.rho;
V = Asteroid.Polyhedron.Vertices;
F = Asteroid.Polyhedron.Facets;
E = Asteroid.Polyhedron.Edges;
F_tilde = Asteroid.Polyhedron.F_tilde;
E_tilde = Asteroid.Polyhedron.E_tilde;
w_AI = Asteroid.w_AI;
mu = Asteroid.mu;

%%-- SC properties--%%
v = SC.Polyhedron.Vertices;
n = SC.Polyhedron.normalsf;
c = SC.Polyhedron.C;
A = SC.Polyhedron.A_facet;
I = SC.I.I_total;
m = SC.mass.m_i;
a_r = SC.a_r;
a_d = SC.a_d; 
vv =[v;c];


%%-- Sun properties--%%
r_d = Sun.pos;

%%-- Switches--%%
BP = Switch.Third_body;

%% Initial State values

kep_orbit(1,1) = 315e3; %a
kep_orbit(1,2) = 0.01; %e
kep_orbit(1,3) = 55; %i
kep_orbit(1,4) = 0; %RAAN
kep_orbit(1,5) = 0; %omega
kep_orbit(1,6) = 0; %M


[state,theta,state_p,theta_p] = kep2cart(kep_orbit,mu);
 x0 = state(1:3,:);
 v0 = state(5:7,:);
 q_AI0 = [0; 0; 0; 1];


tic
%% Integrator 

         T = [0 5*86400];
         Y0 = [[x0(1:3,:); 0]; [v0(1:3,:);0]; q_AI0];    %initial state vector
         
         
%          Opt = odeset('Event',@stopprop,'AbsTol',1e-10,'RelTol',2.3e-10);
         Opt = odeset('AbsTol',1e-2,'RelTol',2.3e-2);
         [t,y] = ode45(@orb_int, T, Y0,Opt); 
    
 
%% Function for state differential

 function dY = orb_int(T,Y)
    %% Velocities
    Y1 = [Y(5); Y(6); Y(7); Y(8)];
    %% Accelerations

    r_A = quat_trans(Y(9:12),Y(1:4),'vect')';
    [~,g_I,Wf,~] = Poly_g_new(r_A,Y(9:12),rho,V,F,E,F_tilde,E_tilde);
%     [a_3BP_I] = F_3BP(Y(1:4),r_d,mu_s,BP);
 

    Y2 = [(g_I(1)); (g_I(2)); (g_I(3)); 0];

    %% Asteroid dynamics

    Wa = [0 w_AI(3) -w_AI(2) w_AI(1)
         -w_AI(3) 0 w_AI(1) w_AI(2)
         w_AI(2) -w_AI(1) 0 w_AI(3)
         -w_AI(1) -w_AI(2) -w_AI(3) 0];

    q_AI_dot = 0.5*Wa*Y(9:12);

    Y3 = q_AI_dot;

    %% State
    dY = [Y1; Y2; Y3];


 end


end