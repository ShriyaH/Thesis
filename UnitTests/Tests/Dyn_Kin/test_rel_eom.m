function [t,y] = test_rel_eom(T_D,I,mu,w_AI,W_AI,x,v,w,q)
%% Initial Values
I_inv = inv(I);

%% Integrator
Y0 = [x';v';w';q'];
T = [0 20000];

%Opt = odeset('Events', @stopevent);
tic
Opt = odeset('RelTol',1e-8,'AbsTol',1e-10);
[t,y] = ode45(@orb_int, T, Y0,Opt); 

toc
%% Differential Function

function dY = orb_int(T,Y)
    r = norm(Y(1:3));
    a_A = -mu.*Y(1:3)/r^3 - 2*W_AI*Y(4:6) - W_AI*(W_AI*Y(1:3)); %no angular acceleration since constant rotation rate
    
    q_BA = Y(10:13)./norm(Y(10:13)); %normalize quaternion
    C_BA = Q2DCM(q_BA);
    
    W_BA = omega_tensor(Y(7:9),2);
    q_BA_dot = 0.5*W_BA*q_BA; %q_BA dot
    

    w_BA_dot = I_inv * (T_D' - cross(Y(7:9),(I*Y(7:9))) - cross(C_BA*w_AI',(I*C_BA*w_AI'))) + cross(Y(7:9),(C_BA*w_AI')); %omega_BA dot

    dY = [Y(4:6); a_A; w_BA_dot; q_BA_dot];
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

% figure()
% comet3(y(:,1),y(:,2),y(:,3));
% hold on
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z (m)');
% grid on
% 
% 
% figure()
% plot(t,y(:,1),'Linewidth',2); 
% hold on
% goodplot
% plot(t,y(:,2),'Linewidth',2); 
% plot(t,y(:,3),'Linewidth',2);
% xlabel('Time (s)');
% ylabel('Position (m)');
% legend('X_x','X_y','X_z','Location','northeastoutside');
% grid on
% 
% figure()
% plot(t,y(:,10),'Linewidth',2); 
% hold on
% goodplot
% plot(t,y(:,11),'Linewidth',2); 
% plot(t,y(:,12),'Linewidth',2);
% plot(t,y(:,13),'Linewidth',2);
% xlabel('Time (s)');
% ylabel('Quaternion Components');
% legend('q_1','q_2','q_3','q_4','Location','northeastoutside');
% grid on
% 
% figure()
% plot(t,y(:,7),'Linewidth',2); 
% hold on
% goodplot
% plot(t,y(:,8),'Linewidth',2); 
% plot(t,y(:,9),'Linewidth',2);
% xlabel('Time (s)');
% ylabel('Omega (rad/s)');
% legend('\omega_x','\omega_y','\omega_z','Location','northeastoutside');
% grid on

end