function [t,y,r_B] = test_rel_b_eom(T_D,w_AI,x,v,w,q)
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
Y0 = [x';v';w';q'];
T = [0 20000];

%Opt = odeset('Events', @stopevent);
tic
Opt = odeset('RelTol',1e-11,'AbsTol',1e-12);
[t,y] = ode45(@orb_int, T, Y0,Opt); 
toc

for i = 1:length(y)
    C(3*i-2:3*i,1:3)  = Q2DCM(y(i,10:13)');
    r_B(i,1:3) = (C(3*i-2:3*i,1:3)* y(i,1:3)')';
%     v_A(i,1:3) = (C(3*i-2:3*i,1:3)* y(i,4:6)')';                                                                                                                                                       
end

%% Differential Function

function dY = orb_int(T,Y)
 
    q_BA = Y(10:13)./norm(Y(10:13)); %normalize quaternion
  
    C_BA = Q2DCM(q_BA);
    C_AB = Q2DCM(conj_quat(q_BA));
    
    w_AI_B = C_BA * w_AI';
    W_AI_B = omega_tensor(w_AI_B,1);
    W_BA1 = omega_tensor(Y(7:9),1);
    W_BA2 = omega_tensor(Y(7:9),2);
    
    W = w_AI_B + Y(7:9);
    WW = omega_tensor(W,1);
    
    q_BA_dot = 0.5*W_BA2*q_BA; %q_BA dot
         
    r_B = C_BA*Y(1:3);
    r2 = norm(r_B);
    
    rs_B = C_BA*Sun.rs_I(1:3); %%warning not correct
    e_B = rs_B./norm(rs_B);
    
    [F_D, T_D] = Get_pertforces(C_BA,r_B,e_B,rs_B,mu);
    a_D = F_D./SC.mass.m_i;
    
    a_B = a_D' - 2*W_AI_B*Y(4:6) - W_BA1*Y(4:6) - W_AI_B*(W_AI_B*(r_B)); %no angular acceleration since constant rotation rate
    
%     w_BA_dot = I_inv * ([0;0;3*mu/r2^3] - W_BA1*(I*Y(7:9)) - W_AI_B*(I*w_AI_B)) - W_AI_B*Y(7:9); %omega_BA dot

    w_BA_dot = I_inv * (T_D - WW*(I*W)) - W_AI_B*Y(7:9); %omega_BA dot
    
    dY = [C_AB*Y(4:6); a_B; w_BA_dot; q_BA_dot]; 
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