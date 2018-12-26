function [t,y,r_At,r_Bt] = test_rel_dq_eom(Sun,dI,mu,w_AI,dw_rel,dq,dq_rel)
%% Initial Values
[U, S, V] = svd(dI);
dI_inv = V*(S\U');
B = [eye(4) zeros(4,4)
    zeros(4,4) 2.*eye(4)]; 
B(8,8) = 1;
w_AI = w_AI';

%% Integrator
Y0 = [dq';dw_rel'];
T = [0 20000];

%Opt = odeset('Events', @stopevent);
tic
Opt = odeset('RelTol',1e-10,'AbsTol',1e-12);
[t,y] = ode45(@orb_int_dqin, T, Y0, Opt); 
toc

codegen -config:mex convt_r -args {zeros(length(y),16,'double')}
[r_At,r_Bt] = convt_r_mex(y);


%% Differential Function
function dY = orb_int_dqin(T,Y)       
    m = 2108.3836;
    T_D = [0; 0; 0; 0];
    
    Y(1:8) = norm_dq(Y(1:8));
    
    dq_W = omega_tensor(Y(9:16),3); %dq cross product matrix
    dq_dot = 0.5*dq_W*Y(1:8); 
    
%     d_W = omega_tensor(Y(9:16),4); %dual number cross product matrix
    C_BA = Q2DCM(Y(1:4));
    
    w_AI_B = C_BA * w_AI; 
    dw_AI_B = [w_AI_B; zeros(5,1)];
    W_AI_B = omega_tensor(dw_AI_B,4);
    
    dw_BI_B = dw_AI_B + Y(9:16);
    W_BI = omega_tensor(dw_BI_B,4);
    
%     W_AI_B = omega_tensor(dq_w_AI_B,4);
%     W_AI_B2 =  W_AI_B;
%     W_AI_B2(1:4,1:4) = [w_AI_B(1) 0 0 0; 0 w_AI_B(2) 0 0; 0 0 w_AI_B(3) 0; 0 0 0 0];

    r_B = 2.*cross_quat(Y(5:8),conj_quat(Y(1:4)));
    r = norm(r_B);
    
    C_AI = Q2DCM([0;0;sin((w_AI(3)*T)/2);cos((w_AI(3)*T)/2)]);
    rs_B = C_BA*C_AI*Sun.rs_I(1:3);
    e_B = rs_B./norm(rs_B);
    
    [F_D,T_D] = Get_pertforces(C_BA,r_B,e_B,rs_B,mu);
%     F_B = -(m*mu).*r_B(1:4)./r^3;
    dF_B = [F_D; T_D];
    
    A = [zeros(4,1); r_B(1:4)];
    
    dw_dot = dI_inv * (dF_B - W_BI*(dI*dw_BI_B) - W_AI_B*(W_AI_B*(dI*A))) - W_AI_B * Y(9:16); %omega_BA dot

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

%% Plots
if dq_rel == 1
figure()
plot3(r_At(:,1),r_At(:,2),r_At(:,3),'Linewidth',2);
hold on
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on

figure ()
plot3(r_Bt(:,1),r_Bt(:,2),r_Bt(:,3),'Linewidth',2);
hold on
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on

figure()
plot(t,r_At(:,1),'Linewidth',2); 
hold on
goodplot
plot(t,r_At(:,2),'Linewidth',2); 
plot(t,r_At(:,3),'Linewidth',2);
xlabel('Time (s)');
ylabel('Position (m)');
legend('X_x','X_y','X_z','Location','northeastoutside');
title('r^A (DQ-Integrator)');
print -depsc relposda

figure()
plot(t,r_Bt(:,1),'Linewidth',2); 
hold on
goodplot
plot(t,r_Bt(:,2),'Linewidth',2); 
plot(t,r_Bt(:,3),'Linewidth',2);
xlabel('Time (s)');
ylabel('Position (m)');
legend('X_x','X_y','X_z','Location','northeastoutside');
title('r^B (DQ-Integrator)');
print -depsc relposdb

figure()
plot(t,y(:,1),'Linewidth',2); 
hold on
goodplot
plot(t,y(:,2),'Linewidth',2); 
plot(t,y(:,3),'Linewidth',2);
plot(t,y(:,4),'Linewidth',2);
xlabel('Time (s)');
ylabel('Quat. Components');
legend('q_1','q_2','q_3','q_4','Location','northeastoutside');
title('q_{B/A} (DQ-Integrator)');
grid on
print -depsc relquatd

figure()
plot(t,y(:,9),'Linewidth',2); 
hold on
goodplot
plot(t,y(:,10),'Linewidth',2); 
plot(t,y(:,11),'Linewidth',2);
xlabel('Time (s)');
ylabel('Omega (rad/s)');
legend('\omega_x','\omega_y','\omega_z','Location','northeastoutside');
title('\omega_{B/A}^B (DQ-Integrator)');
grid on
print -depsc relomegadb

figure()
plot(t,y(:,13),'Linewidth',2); 
hold on
goodplot
plot(t,y(:,14),'Linewidth',2); 
plot(t,y(:,15),'Linewidth',2);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('v_x','v_y','v_z','Location','northeastoutside');
title('v_{B/A}^B (DQ-Integrator)');
grid on
print -depsc relveldb
end
end