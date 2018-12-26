function [t,y,I_total_n,m_n,ch,a_3BP_I] = trans_kin_dyn (T,Sun,Asteroid,Switch,SC,Samples,n_v,n_c)
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
m = SC.mass.wet;
a_r = SC.a_r;
a_d = SC.a_d; 


%%-- Sun properties--%%
f = Sun.flux;
p = Sun.pos;
r_d = Sun.pos;

%%-- Switches--%%
BP = Switch.Third_body;
SRP = Switch.F_SRP;
control = Switch.Control;
grav_model = Switch.Grav_grad;

%% Initial State values
 x0 = Samples.D_sph_init(1:3,:,:);
 v0 = Samples.D_sph_init(4:6,:,:);
 q_BI0 = [0; 0; 0; 1];
 q_AI0 = [0; 0; 0; 1];
 w_BI0 = [0; 0; 0; 0];

tic
%% Integrator 
 for k = 1:n_v          %for number of velocity samples
     I_inv = inv(I);    %inverse of SC inertia
     m_n(1) = m;
     for j = 1:n_c      %for number of state samples
         T = [0 1000];
         Y0 = [[x0(1:3,j,k); 0]; [v0(1:3,j,k);0]; q_AI0];    %initial state vector
         
         
%          Opt = odeset('Event',@stopprop,'AbsTol',1e-10,'RelTol',2.3e-10);
         Opt = odeset('AbsTol',1e-4,'RelTol',2.3e-4);
         [t{k}{j},y{k}{j}] = ode45(@orb_int, T, Y0,Opt); 
     end
     [I_total_n(3*k-2:3*k,1:3),m_n(k+1),ch(3*k-2:3*k,1:3)] = SC_inertia_update(SC,Samples.mag_v(k),I);
 end
toc
 
%% Function for state differential

 function dY = orb_int(T,Y)
    %% Velocities
    Y1 = [Y(5); Y(6); Y(7); Y(8)];
    
    %% Accelerations
    
    r_A = quat_trans(Y(9:12),Y(1:4),'vect')';
    [~,g_I,Wf,~] = F_GG(r_A,Y(9:12),rho,V,F,E,F_tilde,E_tilde,G);
    [a_3BP_I] = F_3BP(Y(1:4),r_d,mu_s,BP);
%     [~,a_SRP_I] = F_SRP(Y(9:12),f,p,n,A,a_r,a_d,AU,vel_light,m_n,SRP);

    Y2 = [(g_I(1)+a_3BP_I(1)); (g_I(2)+a_3BP_I(2)); (g_I(3)+a_3BP_I(3)); 0];
   
%     %% Torques
%     tic
%     for i = 1:length(c)
%     r(:,i) = quat_trans(Y(13:16),c(i,:), 'n');
%     T_SRP(1:3,i) = cross(r(1:3,i),F_SRP_B(i,:)');
%         if grav_model == 1
%             [T_GG_T] = T_GG(v,c,Y(1:3,1),Y(13:16,1),Y(17:20,1),m_n,rho,V,F,E,F_tilde,E_tilde,G);
%         else
%             T_GG_T = 0;
%         end
%     end
%     T_SRP_T = sum(T_SRP')';
%     
%     %Point Mass SC   
%     T_D = T_SRP_T + T_GG_T;
%     toc
%     %% Attitude dynamics
%     tic
%     if control == 1
%         [T_C]= PD_control(T_D,K_P,K_D,w_BI,q_BI); %Control torque
%         w_BI_dot = I_inv * (cross(Y(9:11), I*Y(9:11)) + T_D + T_C);
%     else
%         w_BI_dot = I_inv * (cross(Y(9:11), I*Y(9:11)) + T_D);
%     end
% 
%     Y3 = [w_BI_dot; 0];
%     toc
%     %% Attitude kinematics
%     tic
%     W = [0 Y(11) -Y(10) Y(9)
%          -Y(11) 0 Y(9) Y(10)
%          Y(10) -Y(9) 0 Y(11)
%          -Y(9) -Y(10) -Y(11) 0];
% 
%      
%     q_BI_dot = 0.5*W*Y(13:16);
% 
%     Y4 = q_BI_dot;
%     toc
    %% Asteroid dynamics
    
    Wa = [0 w_AI(3) -w_AI(2) w_AI(1)
         -w_AI(3) 0 w_AI(1) w_AI(2)
         w_AI(2) -w_AI(1) 0 w_AI(3)
         -w_AI(1) -w_AI(2) -w_AI(3) 0];

    q_AI_dot = 0.5*Wa*Y(9:12);

    Y5 = q_AI_dot;

    %% State
    dY = [Y1; Y2; Y5];


 end


% Function for stop event of integrator
% function [value, isterminal, direction] = stopprop(T,Y)
% imp = abs(Wf);                    %Instantaneous ellipsoid wrt Asteroid frame
% 
% r = sqrt(Y(1)^2+Y(2)^2+Y(3)^2);
% esc = sqrt((mu*Y(1)/r^3)^2 + (mu*Y(2)/r^3)^2 + (mu*Y(3)/r^3)^2);  %Instantaneous acceleration
% esc2 = sqrt(3*mu/esc)/4;                                          %Maximum allowable acceleration at that position
% 
% value = [imp-4*pi; esc2-esc]    %Impact condition, Escape condition
% % if abs(value)<=0
% %     value = 0;
% % end
% isterminal = [1; 1];          %Stop the integration
% direction  = [0; 0];          %Bi-directional approach
% end

end