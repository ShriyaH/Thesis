function [t,y] = test_inert_nonlineom(I,m,x,v,q,w,Th,Th_dot,g)
global ITR Switch

%% Initial Values
I_inv = inv(I);
alpha0 = 0.1;
r_T = [-1;0;0];

if Switch.SCvx.check
    SCvx_descent_script_acik;
    xdot_k = ITR.xdot_k{11};
    vc = ITR.v_k{11};
end

%% Integrator
Y0 = [m;x;v;q;w;Th;Th_dot];
T = [0 7];

Opt = odeset('Events', @stopevent);
% Opt = odeset('RelTol',1e-10,'AbsTol',1e-12);
[t,y] = ode45(@orb_int, T, Y0,Opt); 
% 
% if Switch.SCvx.check
%     tspan = linspace(0,5,30);
%     y = ode4(@orb_int,tspan,Y0);  
% end


%% Differential Function

function dY = orb_int(t,Y)

    tt = linspace(0,5,30);
    mm = Y(1,1);
    r = Y(2:4,1);
    vv = Y(5:7,1);
    qq = Y(8:11,1)./norm(Y(8:11,1));
    C = Q2DCM(conj_quat(qq));
    ww = Y(12:14,1);
    W = omega_tensor(ww,2);
    TT = Y(15:17,1);
    Tdot_k = Y(18:20,1);
    
    mdot_k = -alpha0.*norm(TT);
    rdot_k = vv;
    vdot_k = C*TT./mm + g;
    qdot_k = 0.5*W*qq;
    wdot_k = I_inv*(cross(r_T',TT)' - cross(ww, (I*ww)));
%     Tdot_k = -g.*mdot_k;
    u_k = [0;0;0];
    
    dY = [mdot_k;rdot_k;vdot_k;qdot_k;wdot_k;Tdot_k;u_k]  ;  
    
    if Switch.SCvx.check
       [ d, ix ] = min( abs( tt-t ) );
%        u_k = u(1:3,ix);
%        vc = vk(1:20,ix);
       if ix<30
            dY = xdot_k(:,ix);
       else
           dY = zeros(20,1);
       end
    end
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
if Switch.SCvx.check
    for i = 1:30
    qn(i) = norm(y(i,8:11));
        if y(i,14) < 0 
            wn(i) = -rad2deg(norm(y(i,12:14)));
        else
            wn(i) = rad2deg(norm(y(i,12:14)));
        end
%     wn(i) = rad2deg(norm(y(i,12:14)));
    Tn(i) = norm(y(i,15:17));
    rn(i) = norm(y(i,2:3));
    end
    
    figure()
    plot(tspan,y(:,1),'Linewidth',2); 
    xlabel('Time (s)');
    ylabel('Mass (kg)');
    legend('mass','Location','northeastoutside');
    title('m (Non-Linear Dynamics)');
    grid on
%     print -depsc mass_nlin
    
    figure()
    plot(tspan,rn,'Linewidth',2); 
    xlabel('Time (s)');
    ylabel('pos norm');
    legend('r','Location','northeastoutside');
    title('pos norm (Non-Linear Dynamics)');
    grid on
    
    figure()
    plot(tspan,qn,'Linewidth',2); 
    xlabel('Time (s)');
    ylabel('quat norm');
    legend('q','Location','northeastoutside');
    title('quat norm (Non-Linear Dynamics)');
    grid on
    
    figure()
    plot(tspan,wn,'Linewidth',2); 
    xlabel('Time (s)');
    ylabel('ang rate (rad/s)');
    legend('omega','Location','northeastoutside');
    title('ang rate  (Non-Linear Dynamics)');
    grid on
    
    figure()
    plot(tspan,Tn,'Linewidth',2); 
    xlabel('Time (s)');
    ylabel('thrust (N)');
    legend('thrust','Location','northeastoutside');
    title('thrust (Non-Linear Dynamics)');
    grid on

%     figure()
%     comet3(y(:,2),y(:,3),y(:,4));
%     hold on
%     xlabel('X (m)');
%     ylabel('Y (m)');
%     zlabel('Z (m)');
%     grid on
% %     print -depsc orb_nlin

%     figure()
%     plot3(y(1:30,3),y(1:30,4),y(1:30,2),'Color','r');
%     hold on
%     plot3(xc(3,1:30),xc(4,1:30),xc(2,1:30),'Color','b');
%     xlabel('Up (m)');
%     ylabel('East (m)');
%     zlabel('North (m)');
%     grid on

%     for i = 1:30
%         d(1,i) = norm(y(i,2:4)'-xc(2:4,i));
%     end
%     figure()
%     plot(1:30,d)
%     xlabel('Time (secs)');
%     ylabel('Difference in Position Norm (m)');
%     grid on
end
if Switch.Q_plots   
    figure()
    plot(t,y(:,3),'Linewidth',2); 
    hold on
    goodplot()
    plot(t,y(:,4),'Linewidth',2); 
    plot(t,y(:,2),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('Up','East','North','Location','northeastoutside');
    title('r^I (Non-Linear Dynamics)');
    grid on
%     print -depsc pos_nlin
%     
    figure()
    plot(t,y(:,6),'Linewidth',2); 
    hold on
    goodplot()
    plot(t,y(:,7),'Linewidth',2); 
    plot(t,y(:,5),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('v_x','v_y','v_z','Location','northeastoutside');
    title('v^I_{B/I} (Non-Linear Dynamics)');
    grid on
%     print -depsc vel_nlin
% 
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
%     print -depsc qBI_nlin
% 
    figure()
    plot(t,y(:,13),'Linewidth',2); 
    hold on
    goodplot()
    plot(t,y(:,14),'Linewidth',2); 
    plot(t,y(:,12),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Omega (rad/s)');
    legend('\omega_x','\omega_y','\omega_z','Location','northeastoutside');
    title('\omega^B_{B/I} (Non-Linear Dynamics)');
    grid on
%     print -depsc omega_nlin
% 
    figure()
    plot(t,y(:,16),'Linewidth',2); 
    hold on
    goodplot()
    plot(t,y(:,17),'Linewidth',2); 
    plot(t,y(:,15),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Thrust Components');
    legend('T_1','T_2','T_3','Location','northeastoutside');
    title('T_B (Non-Linear Dynamics)');
    grid on
%     print -depsc T_B_nlin

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
%     print -depsc dT_B_nlin

    figure()
    comet3(y(:,3),y(:,4),y(:,2));
    hold on
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on
%     print -depsc orb_nlin
end
end