function [tspan,tode4,y] = test_inert_lineom(I,m,x,v,q,w,Th,Th_dot,g,tl,yl,q_inert)
%% Initial Values
global k
I_inv = inv(I);
alpha0 = 0.1;
r_T = [0;0;-1];

%% Integrator
y0 = [m;x;v;q;w;Th;Th_dot];
K=length(tl)-1;
dt = tl(end)/K;
tspan = 0:dt:tl(end);

tic
y = ode4(@orb_int,tspan,y0);
toc

%% Differential Function

function dy = orb_int(t,y)
    tode4 = t;
    k = t/dt;
    m_k = ((K-k-1)/(K-1))*yl(1,1) + (k/(K-1))*yl(end,1);
    r_k = ((K-k-1)/(K-1)).*yl(1,2:4)' + (k/(K-1))*yl(end,2:4)';
    v_k = ((K-k-1)/(K-1)).*yl(1,5:7)' + (k/(K-1))*yl(end,5:7)';
    q_k = q;
    w_k = w;
    C_k =  Q2DCM(conj_quat(q_k)); 
    T_k = -(m_k*quat_trans(q_k,g,'vect'))';
    Tdot_k = [0;0;0];

    
    x_k  = [m_k;r_k;v_k;q_k;w_k;T_k;Tdot_k];
    
    mdot_k = -alpha0*norm(T_k);
    rdot_k = v_k;
    vdot_k = (C_k*T_k)./m_k + g;
    qdot_k = 1/2.*(omega_tensor(w_k,2)*q_k);
    wdot_k = I_inv*(cross(r_T',T_k')' - cross(w_k, (I*w_k)));
    Tdot_k = g*mdot_k;
    u_k = zeros(3,1);
   
    xdot_k = [mdot_k;rdot_k;vdot_k;qdot_k;wdot_k;Tdot_k;u_k]; 
    
    
    A_k = zeros(20,20);
    A_k(1,15:17) = -alpha0.*(T_k'./norm(T_k));
    A_k(2:4,5:7) = eye(3);
    A_k(5:7,:) = get_da_inertial(T_k,q_k,m_k,20);
    A_k(8:11,:) = get_dqdot_inertial(w_k,q_k,20);
    A_k(12:14,:) = get_dwdot_inertial(w_k,I,r_T,20);
    A_k(15:17,18:20) = eye(3);
    
    B_k = zeros(20,3);
    B_k(18:20,:) = eye(3); 
    z_k = xdot_k - A_k*x_k - B_k*u_k;
   
    y(8:11,1) = y(8:11,1)./norm(y(8:11,1));
    dy = A_k*y(1:20,1) + B_k*u_k + z_k;    
    
end

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
if q_inert == 1
    figure()
    plot(tspan,y(:,1),'Linewidth',2); 
    xlabel('Time (s)');
    ylabel('Mass (kg)');
    legend('mass','Location','northeastoutside');
    title('m (Linearised Dynamics)');
    grid on
    print -depsc mass_lin
    
    figure()
    comet3(y(:,2),y(:,3),y(:,4));
    hold on
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on
    print -depsc orb_lin

    figure()
    plot(tspan,y(:,2),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,y(:,3),'Linewidth',2); 
    plot(tspan,y(:,4),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('X_x','X_y','X_z','Location','northeastoutside');
    title('r^I (Linearised Dynamics)');
    grid on
    print -depsc pos_lin
    
    figure()
    plot(tspan,y(:,5),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,y(:,6),'Linewidth',2); 
    plot(tspan,y(:,7),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('v_x','v_y','v_z','Location','northeastoutside');
    title('v^I_{B/I} (Linearised Dynamics)');
    grid on
    print -depsc vel_lin

    figure()
    plot(tspan,y(:,8),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,y(:,9),'Linewidth',2); 
    plot(tspan,y(:,10),'Linewidth',2);
    plot(tspan,y(:,11),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Quat. Components');
    legend('q_1','q_2','q_3','q_4','Location','northeastoutside');
    title('q_{B/I} (Linearised Dynamics)');
    grid on
    print -depsc qBI_lin

    figure()
    plot(tspan,y(:,12),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,y(:,13),'Linewidth',2); 
    plot(tspan,y(:,14),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Omega (rad/s)');
    legend('\omega_x','\omega_y','\omega_z','Location','northeastoutside');
    title('\omega^B_{B/I} (Linearised Dynamics)');
    grid on
    print -depsc omega_lin

    figure()
    plot(tspan,y(:,15),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,y(:,16),'Linewidth',2); 
    plot(tspan,y(:,17),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Thrust Components');
    legend('T_1','T_2','T_3','Location','northeastoutside');
    title('T_B (Linearised Dynamics)');
    grid on
    print -depsc T_B_lin
    
    figure()
    plot(tspan,y(:,18),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,y(:,19),'Linewidth',2); 
    plot(tspan,y(:,20),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Thrust_dot Components');
    legend('dT_1','dT_2','dT_3','Location','northeastoutside');
    title('dT_B (Linearised Dynamics)');
    grid on
    print -depsc dT_B_lin
end
end