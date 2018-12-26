%% Initial Values

g = [0;0;-1];
I = 0.5.*eye(3);
I_inv = inv(I);
alpha0 = 0.1;
r_T = [0;0;-1];

K=40;
dt = 6.24500000000003/41;
tspan = 0:dt:6.24500000000003;

%% Integrator

x = zeros(20,41);
x(:,1) = [2; 1; 0; 2; 0.2; 0; -1; 0; 0; 0; 1; 0; 0; 0; 0; 0; 2; 0; 0; 0]; 
for k = 0:40
    kk = k+2;
    
    m_k = ((K-k-1)/(K-1))*2 + (k/(K-1))*0.751;
    r_k = ((K-k-1)/(K-1)).*[1;0;2] + (k/(K-1))*[0;0;0];
    v_k = ((K-k-1)/(K-1)).*[0.2;0;-1] + (k/(K-1))*[0;0;-0.1];
    q_k = [0 0 0 1]';
    w_k = zeros(3,1);
    C_k =  Q2DCM(conj_quat(q_k)); 
    T_k = -(m_k*quat_trans(q_k,g,'vect'))';
    Tdot_k = zeros(3,1);
    
    x_k  = [m_k;r_k;v_k;q_k;w_k;T_k;Tdot_k];
    
    mdot_k = -alpha0*norm(T_k);
    rdot_k = v_k;
    vdot_k = (C_k*T_k)./m_k + g;
    qdot_k = 1/2.*(omega_tensor(w_k,2)*q_k);
    wdot_k = I_inv*(cross(r_T',T_k')' - cross(w_k, (I*w_k)));
    u_k = zeros(3,1);
   
    xdot_k = [mdot_k;rdot_k;vdot_k;qdot_k;wdot_k;Tdot_k;u_k]; 
    
    
    A_k = zeros(20,20);
    A_k(1,15:17) = -alpha0.*(T_k'./norm(T_k));
    A_k(2:4,5:7) = eye(3);
    A_k(5:7,:) = get_da(T_k,q_k,m_k,20);
    A_k(8:11,:) = get_dqdot(w_k,q_k,20);
    A_k(12:14,:) = get_dwdot(w_k,I,r_T,20);
    A_k(15:17,18:20) = eye(3);
    
    B_k = zeros(20,3);
    B_k(18:20,:) = eye(3); 
    z_k = xdot_k - A_k*x_k - B_k*u_k;
    
    p = 2; %number of terms included in the series expansion 
    Psi = zeros(size(A_k));
    for pp = 0:1:p
        dPsi = (dt^pp/factorial(pp+1))*A_k^pp;
        Psi = Psi+dPsi;
    end
    
    Ad = eye(size(A_k)) + dt.*(A_k*Psi);
    
    Bd = dt.*(Psi*B_k);
    
    zd = dt.*(Psi*z_k);
    
    x(8:11,kk-1) = x(8:11,kk-1)./norm(x(8:11,kk-1));
    x(:,kk) = Ad*x(:,kk-1) + Bd*u_k + zd;    
    

end
x= x';

%% Plots

    figure()
    plot(tspan,x(:,1),'Linewidth',2); 
    xlabel('Time (s)');
    ylabel('Mass (kg)');
    legend('mass','Location','northeastoutside');
    title('m (Discretised Dynamics)');
    grid on
    print -depsc mass_dis
    
    figure()
    comet3(x(:,2),x(:,3),x(:,4));
    hold on
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on
    print -depsc orb_dis

    figure()
    plot(tspan,x(:,2),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,x(:,3),'Linewidth',2); 
    plot(tspan,x(:,4),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('X_x','X_y','X_z','Location','northeastoutside');
    title('r^I (Discretised Dynamics)');
    grid on
    print -depsc pos_dis
    
    figure()
    plot(tspan,x(:,5),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,x(:,6),'Linewidth',2); 
    plot(tspan,x(:,7),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('v_x','v_y','v_z','Location','northeastoutside');
    title('v^I_{B/I} (Discretised Dynamics)');
    grid on
    print -depsc vel_dis

    figure()
    plot(tspan,x(:,8),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,x(:,9),'Linewidth',2); 
    plot(tspan,x(:,10),'Linewidth',2);
    plot(tspan,x(:,11),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Quat. Components');
    legend('q_1','q_2','q_3','q_4','Location','northeastoutside');
    title('q_{B/I} (Discretised Dynamics)');
    grid on
    print -depsc qBI_dis

    figure()
    plot(tspan,x(:,12),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,x(:,13),'Linewidth',2); 
    plot(tspan,x(:,14),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Omega (rad/s)');
    legend('\omega_x','\omega_y','\omega_z','Location','northeastoutside');
    title('\omega^B_{B/I} (Discretised Dynamics)');
    grid on
    print -depsc omega_dis

    figure()
    plot(tspan,x(:,15),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,x(:,16),'Linewidth',2); 
    plot(tspan,x(:,17),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Thrust Components');
    legend('T_1','T_2','T_3','Location','northeastoutside');
    title('T_B (Discretised Dynamics)');
    grid on
    print -depsc T_B_dis
    
    figure()
    plot(tspan,x(:,18),'Linewidth',2); 
    hold on
    goodplot()
    plot(tspan,x(:,19),'Linewidth',2); 
    plot(tspan,x(:,20),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Thrust_dot Components');
    legend('dT_1','dT_2','dT_3','Location','northeastoutside');
    title('dT_B (Discretised Dynamics)');
    grid on
    print -depsc dT_B_dis

