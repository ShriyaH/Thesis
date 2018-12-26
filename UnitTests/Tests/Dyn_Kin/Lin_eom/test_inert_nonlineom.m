function [t,y] = test_inert_nonlineom(I,m,x,v,q,w,Th,Th_dot,g,q_inert)
%% Initial Values
I_inv = inv(I);
alpha0 = 0.1;
r_T = [0;0;-1];

%% Integrator
Y0 = [m;x;v;q;w;Th;Th_dot];
T = [0 7];

Opt = odeset('Events', @stopevent);
% tspan = 0:10:20000;
% t = tspan;
% dt = 10;
tic
%Opt = odeset('RelTol',1e-10,'AbsTol',1e-12);
[t,y] = ode45(@orb_int, T, Y0,Opt); 

toc
%% Differential Function

function dY = orb_int(T,Y)
    mm = Y(1,1);
    r = Y(2:4,1);
    vv = Y(5:7,1);
    qq = Y(8:11,1)/norm(Y(8:11,1));
    C = Q2DCM(conj_quat(qq));
    ww = Y(12:14,1);
    W = omega_tensor(ww,2);
    T = Y(15:17,1);
    Tdot = Y(18:20,1);
    
    mdot_k = -alpha0.*norm(T);
    rdot_k = vv;
    vdot_k = (C*T)./mm + g;
    qdot_k = 0.5*W*qq;
    wdot_k = I_inv*(cross(r_T',T)' - cross(ww, (I*ww)));
    Tdot_k = g*mdot_k;
    u_k = [0;0;0];
    
    dY = [mdot_k;rdot_k;vdot_k;qdot_k;wdot_k;Tdot_k;u_k];    
end

%% Stop Integrator

function [value, isterminal, direction] = stopevent(T,Y)
% imp = sqrt(Y(2)^2 + Y(3)^2 + Y(4)^2);                         % Achieved mass 
imp = Y(1);
value = imp-0.75;                   % Minimum mass
if abs(value)<=1e-3
    value = 0;
end
isterminal = 1;          %Stop the integration
direction  = -1;          %Bi-directional approach
end

%% Plots
if q_inert == 1
    figure()
    plot(t,y(:,1),'Linewidth',2); 
    xlabel('Time (s)');
    ylabel('Mass (kg)');
    legend('mass','Location','northeastoutside');
    title('m (Non-Linear Dynamics)');
    grid on
    print -depsc mass_nlin
    
    figure()
    comet3(y(:,2),y(:,3),y(:,4));
    hold on
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on
    print -depsc orb_nlin

    figure()
    plot(t,y(:,2),'Linewidth',2); 
    hold on
    goodplot()
    plot(t,y(:,3),'Linewidth',2); 
    plot(t,y(:,4),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('X_x','X_y','X_z','Location','northeastoutside');
    title('r^I (Non-Linear Dynamics)');
    grid on
    print -depsc pos_nlin
    
    figure()
    plot(t,y(:,5),'Linewidth',2); 
    hold on
    goodplot()
    plot(t,y(:,6),'Linewidth',2); 
    plot(t,y(:,7),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('v_x','v_y','v_z','Location','northeastoutside');
    title('v^I_{B/I} (Non-Linear Dynamics)');
    grid on
    print -depsc vel_nlin

    figure()
    plot(t,y(:,8),'Linewidth',2); 
    hold on
    goodplot()
    plot(t,y(:,9),'Linewidth',2); 
    plot(t,y(:,10),'Linewidth',2);
    plot(t,y(:,11),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Quat. Components');
    legend('q_1','q_2','q_3','q_4','Location','northeastoutside');
    title('q_{B/I} (Non-Linear Dynamics)');
    grid on
    print -depsc qBI_nlin

    figure()
    plot(t,y(:,12),'Linewidth',2); 
    hold on
    goodplot()
    plot(t,y(:,13),'Linewidth',2); 
    plot(t,y(:,14),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Omega (rad/s)');
    legend('\omega_x','\omega_y','\omega_z','Location','northeastoutside');
    title('\omega^B_{B/I} (Non-Linear Dynamics)');
    grid on
    print -depsc omega_nlin

    figure()
    plot(t,y(:,15),'Linewidth',2); 
    hold on
    goodplot()
    plot(t,y(:,16),'Linewidth',2); 
    plot(t,y(:,17),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Thrust Components');
    legend('T_1','T_2','T_3','Location','northeastoutside');
    title('T_B (Non-Linear Dynamics)');
    grid on
    print -depsc T_B_nlin
    
    figure()
    plot(t,y(:,18),'Linewidth',2); 
    hold on
    goodplot()
    plot(t,y(:,19),'Linewidth',2); 
    plot(t,y(:,20),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Thrust_dot Components');
    legend('dT_1','dT_2','dT_3','Location','northeastoutside');
    title('dT_B (Non-Linear Dynamics)');
    grid on
    print -depsc dT_B_nlin
end
end