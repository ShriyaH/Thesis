function [t,y,r_At,r_Bt,T_g,TT] = test_rel_dq_eom(dI,w_AI,dq_rel,dw_rel)
global Sun Kleopatra Switch Var

%% Initial Values
if Switch.Razgus
    G = 6.67e-11;
    m = 2.618e18;
    mu = G*m; %standard grav parameter
else
   mu = Kleopatra.mu; 
end
[U, S, V] = svd(dI);
dI_inv = V*(S\U');
% w_AI = w_AI';

%% Integrator
Y0 = [dq_rel';dw_rel'];
T = [0 20000];
i =1;
%Opt = odeset('Events', @stopevent);
tic
Opt = odeset('RelTol',1e-10,'AbsTol',1e-12);
[t,y] = ode45(@orb_int_dqin, T, Y0, Opt); 
toc

% codegen -config:mex convt_r -args {zeros(length(y),16,'double')}
% [r_At,r_Bt] = convt_r_mex(y);

[r_At,r_Bt] = convt_r(y);

%% Differential Function
function [dY] = orb_int_dqin(T,Y)   
  
    i = i+1;
    TT(i)=T;
    m = 2108.3836;
    
    Y(1:8) = norm_dq(Y(1:8));
    
    dW_om = omega_tensor(Y(9:16),3); %dq cross product matrix
    dq_dot = 0.5*dW_om*Y(1:8); 

    C_BA = Q2DCM(Y(1:4));
    
    w_AI_B = C_BA * w_AI; 
    dw_AI_B = [w_AI_B; zeros(5,1)];
    W_AI_B = omega_tensor(dw_AI_B,4);
    
    dw_BI_B = dw_AI_B + Y(9:16);
    W_BI = omega_tensor(dw_BI_B,4);

    r_B = DQ2R(Y(1:8),1);
    
    C_AI = Q2DCM([0;0;sin((w_AI(3)*T)/2);cos((w_AI(3)*T)/2)]);
    rs_B = C_BA*C_AI*Sun.rs_I(1:3);
    e_B = rs_B./norm(rs_B);
    
    [F_D,T_D] = Get_pertforces(C_BA,r_B,e_B,rs_B,mu,Kleopatra);
    dF_B = [F_D; T_D];
    T_g(:,i) =T_D;
    A = [zeros(4,1); r_B(1:4)];
    
    dw_dot = dI_inv*(dF_B - W_BI*(dI*dw_BI_B)) - W_AI_B*(W_AI_B*A) - W_AI_B*Y(9:16); %omega_BA dot

    dY = [dq_dot; dw_dot];
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