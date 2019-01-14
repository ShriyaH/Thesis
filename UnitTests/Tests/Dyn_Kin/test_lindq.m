function [tode4,y] = test_lindq(J,m_sc,dq,dw,dF,dF_dot,g,tnl)
%% Initial Values
global k Kleopatra Switch 

%% Initial Values
if Switch.Razgus
    G = 6.67e-11;
    m = 2.618e18;
    mu = G*m; %standard grav parameter
else
   mu = Kleopatra.mu; 
end

alpha0 = 0.1;
r_T = [-1,0,0];

%% Integrator
y0 = [m_sc;dq;dw;dF;dF_dot];

K=length(tnl)-1;
dt = tnl(end)/K;
tspan = 0:dt:tnl(end);

tic
y = ode4(@orb_int,tspan,y0);

%% Convert to r
for i = 1:length(y)
    r_I(1:4,i) = DQ2R(y(i,2:9)',2);
    v_I(1:3,i) = quat_trans(conj_quat(y(i,2:5)'),y(i,14:16),'vect')';
end
toc

%% Differential Function
function dy = orb_int(t,y)
    tode4 = t;
    k = t/dt;
%     [ d, ix ] = min( abs( tt-t ) );
    m_k = y(1);
    dq_k = y(2:9);
    dq_k = norm_dq(dq_k);
    dw_k = y(10:17);
    dF_k = y(18:25);
    dFdot_k = y(26:33);
    
    x_k  = [m_k;dq_k;dw_k;dF_k;dFdot_k];
    
    dJ_k = dq_inertia(m_k,J);
    [U, S, V] = svd(dJ_k);
    dJ_inv_k = V*(S\U');
        
    % construct linearisation point state differential
    mdot_k = -alpha0*norm(dF_k(1:4));
    dqdot_k = 1/2.*(omega_tensor(dw_k,3)*dq_k);
    gb = quat_trans(dq_k(1:4),g,'n');
    dwdot_k = dJ_inv_k*(dF_k - omega_tensor(dw_k,4)*(dJ_k*dw_k) + [m_k.*gb;0;0;0;0]);
%     Fdot_k = mdot_k.*gb;
%     dFdot_k = [Fdot_k;cross(r_T',Fdot_k(1:3));0];
    u_k = [0;0;0;0;0;0;0;0];
    
    xdot_k = [mdot_k;dqdot_k;dwdot_k;dFdot_k;u_k];
    
    A_k = zeros(33,33);
    A_k(1,18:20) = -alpha0.*(dF_k(1:3)'./norm(dF_k(1:3)));
    A_k(2:9,:) = get_dDQdot(dw_k,dq_k,33);
    A_k(10:17,:) = get_dDWdot(m_k,dw_k,dJ_k,dq_k,dF_k,r_T,33);
    A_k(18:25,26:33) = eye(8);
    
    B_k = zeros(33,8);
    B_k(26:33,:) = eye(8);
    
    z_k = xdot_k - A_k*x_k - B_k*u_k;
   
    dy = A_k*y(1:33,1) + B_k*u_k + z_k;    
    
end

% function dy = orb_int(t,y)
%     tode4 = t;
%     k = t/dt;
% %     [ d, ix ] = min( abs( tt-t ) );
%     m_k = ((K-k-1)/(K-1))*ynl(1,1) + (k/(K-1))*ynl(end,1);
%     r_k = ((K-k-1)/(K-1)).*ynl(1,2:4)' + (k/(K-1))*ynl(end,2:4)';
%     q_k = ynl(1,8:11)';
%     C_k =  Q2DCM(conj_quat(q_k)); 
%     dq_k = Q2DQ(q_k,r_k,2);
%     dq_k = norm_dq(dq_k);
%     v_k = ((K-k-1)/(K-1)).*ynl(1,5:7)' + (k/(K-1))*ynl(end,5:7)';
%     v_k = C_k*v_k;
%     w_k = ((K-k-1)/(K-1)).*ynl(1,12:14)' + (k/(K-1))*ynl(end,12:14)';
%     dw_k = [w_k;0;v_k;0];
%     F_k = -(m_k.*quat_trans(q_k,g,'vect'));
%     dF_k = [F_k';0;cross(r_T,F_k)';0];
%     dFdot_k = [0;0;0;0;0;0;0;0];
%     
%     x_k  = [m_k;dq_k;dw_k;dF_k;dFdot_k];
%     
%     dJ_k = dq_inertia(m_k,J);
%     [U, S, V] = svd(dJ_k);
%     dJ_inv_k = V*(S\U');
%         
%     % construct linearisation point state differential
%     mdot_k = -alpha0*norm(dF_k(1:4));
%     dqdot_k = 1/2.*(omega_tensor(dw_k,3)*dq_k);
%     gb = quat_trans(dq_k(1:4),g,'n');
%     dwdot_k = dJ_inv_k*(dF_k - omega_tensor(dw_k,4)*(dJ_k*dw_k) + [m_k.*gb;0;0;0;0]);
% %     Fdot_k = mdot_k.*gb;
% %     dFdot_k = [Fdot_k;cross(r_T',Fdot_k(1:3));0];
%     u_k = [0;0;0;0;0;0;0;0];
%     
%     xdot_k = [mdot_k;dqdot_k;dwdot_k;dFdot_k;u_k];
%     
%     A_k = zeros(33,33);
%     A_k(1,18:20) = -alpha0.*(dF_k(1:3)'./norm(dF_k(1:3)));
%     A_k(2:9,:) = get_dDQdot(dw_k,dq_k,33);
%     A_k(10:17,:) = get_dDWdot(m_k,dw_k,dJ_k,dq_k,dF_k,wa,r_T,33,Kleopatra);
%     A_k(18:25,26:33) = eye(8);
%     
%     B_k = zeros(33,8);
%     B_k(26:33,:) = eye(8);
%     
%     z_k = xdot_k - A_k*x_k - B_k*u_k;
%    
%     y(2:5,1) =  y(2:5,1)./norm(y(2:5,1));
%     dy = A_k*y(1:33,1) + B_k*u_k + z_k;    
%     
% end

%% Stop Integrator

% function [value, isterminal, direction] = stopevent(T,Y)
% % imp = sqrt(Y(2)^2 + Y(3)^2 + Y(4)^2);                         % Achieved mass 
% imp = Y(1);
% value = imp-0.75;                   % Minimum mass
% if abs(value)<=1e-3
%     value = 0;
% end
% isterminal = 1;          %Stop the integration
% direction  = -1;          %Bi-directional approach
% end


%% Plots
if Switch.q_inert
    K=length(tnl)-1;
    dt = tnl(end)/K;
    tspan = 0:dt:tnl(end);
    figure(13)
    plot(tspan,y(:,1),'Linewidth',2); 
    xlabel('Time (s)');
    ylabel('Mass (kg)');
    legend('mass','Location','northeastoutside');
    title('m (Linearised Dynamics)');
    grid on
    %print -depsc mass_lin
    
    figure(14)
    comet3(r_I(2,:),r_I(3,:),r_I(1,:));
    hold on
    xlabel('Up (m)');
    ylabel('East (m)');
    zlabel('North (m)');
    grid on
    %print -depsc orb_lin

    figure(15)
    plot(tspan,r_I(2,:),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,r_I(3,:),'Linewidth',2); 
    plot(tspan,r_I(1,:),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('Up','East','North','Location','northeastoutside');
    title('r^I (Linearised Dynamics)');
    grid on
    %print -depsc pos_lin
    
    figure(16)
    plot(tspan,v_I(2,:),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,v_I(3,:),'Linewidth',2); 
    plot(tspan,v_I(1,:),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('v_x','v_y','v_z','Location','northeastoutside');
    title('v^I_{B/I} (Linearised Dynamics)');
    grid on
    %print -depsc vel_lin

    figure(17)
    plot(tspan,y(:,2),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,y(:,3),'Linewidth',2); 
    plot(tspan,y(:,4),'Linewidth',2);
    plot(tspan,y(:,5),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Quat. Components');
    legend('q_1','q_2','q_3','q_4','Location','northeastoutside');
    title('q_{B/I} (Linearised Dynamics)');
    grid on
    %print -depsc qBI_lin

    figure(18)
    plot(tspan,y(:,11),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,y(:,12),'Linewidth',2); 
    plot(tspan,y(:,10),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Omega (rad/s)');
    legend('\omega_x','\omega_y','\omega_z','Location','northeastoutside');
    title('\omega^B_{B/I} (Linearised Dynamics)');
    grid on
    %print -depsc omega_lin

    figure(19)
    plot(tspan,y(:,19),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,y(:,20),'Linewidth',2); 
    plot(tspan,y(:,18),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Thrust Components');
    legend('T_1','T_2','T_3','Location','northeastoutside');
    title('T_B (Linearised Dynamics)');
    grid on
    %print -depsc T_B_lin
    
    figure(20)
    plot(tspan,y(:,27),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,y(:,28),'Linewidth',2); 
    plot(tspan,y(:,26),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Thrust_dot Components');
    legend('dT_1','dT_2','dT_3','Location','northeastoutside');
    title('dT_B (Linearised Dynamics)');
    grid on
    %print -depsc dT_B_lin
end
end