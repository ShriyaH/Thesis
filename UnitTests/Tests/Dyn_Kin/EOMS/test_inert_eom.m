function [t,y] = test_inert_eom(x,v,w,q1,q2,W2)
global Sun SC Kleopatra Switch
%% Initial Values

if Switch.Razgus
    G = 6.67e-11;
    m = 2.618e18;
    mu = G*m; %standard grav parameter
else
   mu = Kleopatra.mu; 
end
I = SC.I.I_total;
[U, S, V] = svd(I);
I_inv = V*(S\U');

%% Integrator
Y0 = [x';v';w';q1';q2'];
% T = [0 20000];
T = [0 30072]; %for SRP orbit

%Opt = odeset('Events', @stopevent);
tic
Opt = odeset('RelTol',1e-10,'AbsTol',1e-12);
[t,y] = ode45(@orb_int, T, Y0,Opt); 
toc

%% Differential Function

function dY = orb_int(T,Y)
    q_BI = Y(10:13)./norm(Y(10:13)); %normalize
    W1 = omega_tensor(Y(7:9),2);
    C_BI = Q2DCM(q_BI);
    
    q_BI_dot = 0.5*W1*q_BI; %q_BI dot
    
    q_AI = Y(14:17)./norm(Y(14:17)); %normalize
    C_AI = Q2DCM(q_AI);
    q_AI_dot = 0.5*W2*Y(14:17); %q_AI dot
    
    rs_B = C_BI*Sun.rs_I(1:3);
    e_B = rs_B./norm(rs_B)
%     e_B = [-1;0;0];
    r_B = C_BI*Y(1:3);
    C_BA = C_BI*C_AI';
    
    [F_D,T_D, a_SRP_T] = Get_pertforces(C_BA,r_B,e_B,rs_B,mu);
    a_SRP_T
    a_D = F_D/SC.mass.m_i;
    wd = I_inv * (T_D(1:3) - cross(Y(7:9), (I*Y(7:9)))); %omega dot
    
    a_D_I = quat_trans(conj_quat(q_BI),a_D,'n');
    
    dY = [Y(4:6); a_D_I(1:3); wd; q_BI_dot; q_AI_dot];
end

%% Stop Integrator

% function [value, isterminal, direction] = stopevent(T,Y)
% imp = sqrt(Y(1)^2 + Y(2)^2 + Y(3)^2)                                     %Maximum allowable acceleration at that position
% value = imp-6371e3    %Impact condition, Escape condition
% if abs(value)<=1e-2
%     value = 0;
% end
% isterminal = 1;          %Stop the integration
% direction  = -1;          %Bi-directional approach
% end

end