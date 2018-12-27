function []= dyn_block_plots()
global Switch Var

if Switch.q_inert
    figure()
    comet3(Var.y(:,1),Var.y(:,2),Var.y(:,3));
    hold on
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on
    print -depsc inertorb

    figure()
    plot(Var.t,Var.y(:,1),'Linewidth',2); 
    hold on
    goodplot()
    plot(Var.t,Var.y(:,2),'Linewidth',2); 
    plot(Var.t,Var.y(:,3),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('X_x','X_y','X_z','Location','northeastoutside');
    title('r^I (Q-Integrator)');
    grid on
    print -depsc inertpos
    
    figure()
    plot(Var.t,Var.y(:,4),'Linewidth',2); 
    hold on
    goodplot()
    plot(Var.t,Var.y(:,5),'Linewidth',2); 
    plot(Var.t,Var.y(:,6),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('v_x','v_y','v_z','Location','northeastoutside');
    title('v^I_{B/I} (Q-Integrator)');
    grid on
    print -depsc inertvel

    figure()
    plot(Var.t,Var.y(:,10),'Linewidth',2); 
    hold on
    goodplot()
    plot(Var.t,Var.y(:,11),'Linewidth',2); 
    plot(Var.t,Var.y(:,12),'Linewidth',2);
    plot(Var.t,Var.y(:,13),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Quat. Components');
    legend('q_1','q_2','q_3','q_4','Location','northeastoutside');
    title('q_{B/I} (Q-Integrator)');
    grid on
    print -depsc inertqBI

    figure()
    plot(Var.t,Var.y(:,7),'Linewidth',2); 
    hold on
    goodplot()
    plot(Var.t,Var.y(:,8),'Linewidth',2); 
    plot(Var.t,Var.y(:,9),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Omega (rad/s)');
    legend('\omega_x','\omega_y','\omega_z','Location','northeastoutside');
    title('\omega^B_{B/I} (Q-Integrator)');
    grid on
    print -depsc inertomega

    figure()
    plot(Var.t,Var.y(:,14),'Linewidth',2); 
    hold on
    goodplot()
    plot(Var.t,Var.y(:,15),'Linewidth',2); 
    plot(Var.t,Var.y(:,16),'Linewidth',2);
    plot(Var.t,Var.y(:,17),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Quat. Components');
    legend('q_1','q_2','q_3','q_4','Location','northeastoutside');
    title('q_{A/I} (Q-Integrator)');
    grid on
    print -depsc inertqAI
end

if Switch.convert_q
    figure()
        comet3(Var.r_Bt(1,:),Var.r_Bt(2,:),Var.r_Bt(3,:));
        hold on
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        grid on
        print -depsc relorbb

        % figure()
        % comet3(Var.r_At(1,:),Var.r_At(2,:),Var.r_At(3,:));
        % hold on
        % xlabel('X (m)');
        % ylabel('Y (m)');
        % zlabel('Z (m)');
        % grid on
        % print -depsc relorba

        figure()
        plot(Var.t,Var.r_At(1,:),'Linewidth',2); 
        hold on 
        goodplot
        plot(Var.t,Var.r_At(2,:),'Linewidth',2); 
        plot(Var.t,Var.r_At(3,:),'Linewidth',2);
        xlabel('Time (s)');
        ylabel('Position (m)');
        legend('X_x','X_y','X_z','Location','northeastoutside');
        title('r^A (Q-Integrator)');
        grid on
        print -depsc relposac

        figure()
        plot(Var.t,Var.r_Bt(1,:),'Linewidth',2); 
        hold on 
        goodplot
        plot(Var.t,Var.r_Bt(2,:),'Linewidth',2); 
        plot(Var.t,Var.r_Bt(3,:),'Linewidth',2);
        xlabel('Time (s)');
        ylabel('Position (m)');
        legend('X_x','X_y','X_z','Location','northeastoutside');
        title('r^B (Q-Integrator)');
        grid on
        print -depsc relposbc

        figure()
        plot(Var.t,Var.w_BA_b(1,:),'Linewidth',2); 
        hold on
        goodplot
        plot(Var.t,Var.w_BA_b(2,:),'Linewidth',2); 
        plot(Var.t,Var.w_BA_b(3,:),'Linewidth',2);
        xlabel('Time (s)');
        ylabel('Omega (rad/s)');
        legend('\omega_x','\omega_y','\omega_z','Location','northeastoutside');
        title('\omega_{B/A}^B (Q-Integrator)');
        grid on
        print -depsc relomegabc

        figure()
        plot(Var.t,Var.w_BA_a(1,:),'Linewidth',2); 
        hold on
        goodplot
        plot(Var.t,Var.w_BA_a(2,:),'Linewidth',2); 
        plot(Var.t,Var.w_BA_a(3,:),'Linewidth',2);
        xlabel('Time (s)');
        ylabel('Omega (rad/s)');
        legend('\omega_x','\omega_y','\omega_z','Location','northeastoutside');
        title('\omega_{B/A}^A (Q-Integrator)');
        grid on
        print -depsc relomegaac

        figure()
        plot(Var.t,Var.v_At(1,:),'Linewidth',2); 
        hold on
        goodplot
        plot(Var.t,Var.v_At(2,:),'Linewidth',2); 
        plot(Var.t,Var.v_At(3,:),'Linewidth',2);
        xlabel('Time (s)');
        ylabel('Velocity (m/s)');
        legend('v_x','v_y','v_z','Location','northeastoutside');
        title('v_{B/A}^A (Q-Integrator)');
        grid on
        print -depsc relvelac

        figure()
        plot(Var.t,Var.v_Bt(1,:),'Linewidth',2); 
        hold on
        goodplot
        plot(Var.t,Var.v_Bt(2,:),'Linewidth',2); 
        plot(Var.t,Var.v_Bt(3,:),'Linewidth',2);
        xlabel('Time (s)');
        ylabel('Velocity (m/s)');
        legend('v_x','v_y','v_z','Location','northeastoutside');
        title('v_{B/A}^B (Q-Integrator)');
        grid on
        print -depsc relvelbc

        figure()
        plot(Var.t,Var.q_BAt(1,:),'Linewidth',2); 
        hold on
        goodplot()
        plot(Var.t,Var.q_BAt(2,:),'Linewidth',2); 
        plot(Var.t,Var.q_BAt(3,:),'Linewidth',2);
        plot(Var.t,Var.q_BAt(4,:),'Linewidth',2);
        xlabel('Time (s)');
        ylabel('Quat. Components');
        legend('q_1','q_2','q_3','q_4','Location','northeastoutside');
        title('q_{B/A} (Q-Integrator)');
        grid on
        print -depsc relqBAc
        
        figure()
        plot(Var.t,Var.w_AI_B(1,:),'Linewidth',2); 
        hold on
        goodplot
        plot(Var.t,Var.w_AI_B(2,:),'Linewidth',2); 
        plot(Var.t,Var.w_AI_B(3,:),'Linewidth',2);
        xlabel('Time (s)');
        ylabel('Omega (rad/s)');
        legend('\omega_x','\omega_y','\omega_z','Location','northeastoutside');
        title('\omega_{A/I}^B (Q-Integrator)');
        grid on
        print -depsc relomegaaic
end

if Switch.rel
    figure()
    comet3(Var.r_B(:,1),Var.r_B(:,2),Var.r_B(:,3));
    hold on
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on

    figure()
    comet3(Var.y2(:,1),Var.y2(:,2),Var.y2(:,3));
    hold on
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on

    figure()
    plot(Var.t2,Var.y2(:,1),'Linewidth',2); 
    hold on
    goodplot
    plot(Var.t2,Var.y2(:,2),'Linewidth',2); 
    plot(Var.t2,Var.y2(:,3),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('X_x','X_y','X_z','Location','northeastoutside');
    title('r^A (Rel_Q-Integrator)');
    grid on

    figure()
    plot(Var.t2,Var.r_B(:,1),'Linewidth',2); 
    hold on
    goodplot
    plot(Var.t2,Var.r_B(:,2),'Linewidth',2); 
    plot(Var.t2,Var.r_B(:,3),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('X_x','X_y','X_z','Location','northeastoutside');
    title('r^B (Rel_Q-Integrator)');
    grid on
    print -depsc relposb
    
    figure()
    plot(Var.t2,Var.y2(:,4),'Linewidth',2); 
    hold on
    goodplot
    plot(Var.t2,Var.y2(:,5),'Linewidth',2); 
    plot(Var.t2,Var.y2(:,6),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('X_x','X_y','X_z','Location','northeastoutside');
    title('v_{B/A}^B (Rel_Q-Integrator)');
    grid on
    print -depsc relvelb
    
    figure()
    plot(Var.t2,Var.y2(:,10),'Linewidth',2); 
    hold on
    goodplot
    plot(Var.t2,Var.y2(:,11),'Linewidth',2); 
    plot(Var.t2,Var.y2(:,12),'Linewidth',2);
    plot(Var.t2,Var.y2(:,13),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Quat. Components');
    legend('q_1','q_2','q_3','q_4','Location','northeastoutside');
    title('q_{B/A} (Rel_Q-Integrator)');
    grid on
    print -depsc relquat

    figure()
    plot(Var.t2,Var.y2(:,7),'Linewidth',2); 
    hold on
    goodplot
    plot(Var.t2,Var.y2(:,8),'Linewidth',2); 
    plot(Var.t2,Var.y2(:,9),'Linewidth',2);
    xlabel('Time (s)');
    ylabel('Omega (rad/s)');
    legend('\omega_x','\omega_y','\omega_z','Location','northeastoutside');
    title('\omega_{B/A}^B (Rel_Q-Integrator)');
    grid on
    print -depsc relomegab
end

if Switch.kepler_n
    for i = 1:length(y)
        [Kepler(i,:),Kepler_deg(i,:)] = cart2kep(y(i,1:3),y(i,4:6),mu);
    end
    figure()
    goodplot
    hold on
    plot (t,Kepler_deg(:,1),'Linewidth',2)
    xlabel('Time (secs)');
    ylabel('Metres');
    grid on

    figure()
    plot (t,Kepler_deg(:,3),'Linewidth',2)
    hold on
    goodplot
    plot (t,-Kepler_deg(:,4),'Linewidth',2)
    xlabel('Time (secs)');
    ylabel('Degrees');
    legend('i','\Omega','Location','northeastoutside');
    grid on
end

if Switch.kepler_el
   for k = 1:length(r_I)
            [Kepler(k,:),Kepler_deg(k,:)] = cart2kep(r_I(k,1:3),v_I(k,1:3),mu);
        end
        figure()
        plot (t3,Kepler_deg(:,1),'Linewidth',2)
        hold on 
        goodplot
        xlabel('Time (secs)');
        ylabel('Metres');
        title('Semi-major Axis (DQ-Integrator)');
        legend('a','Location','northeastoutside');
        axis([0 2e4 99999 1.00001e5])
        grid on
        print -depsc kepa

        figure()
        plot (t3,Kepler_deg(:,3),'Linewidth',2)
        hold on
        goodplot
        plot (t3,Kepler_deg(:,4),'Linewidth',2)
        xlabel('Time (secs)');
        ylabel('Degrees');
        legend('i','\Omega','Location','northeastoutside');
        title('Inclination and RAAN (DQ-Integrator)');
        axis([0 2e4 89.9995 90.001])
        grid on
        print -depsc kepio 
end
%         figure()
%         plot(t3,w_AI_Btt(1,:),'Linewidth',2); 
%         hold on
%         goodplot
%         plot(t3,w_AI_Btt(2,:),'Linewidth',2); 
%         plot(t3,w_AI_Btt(3,:),'Linewidth',2);
%         xlabel('Time (s)');
%         ylabel('Omega (rad/s)');
%         legend('\omega_x','\omega_y','\omega_z','Location','northeastoutside');
%         title('\omega_{A/I}^B (Q-Integrator)');
%         grid on
%         print -depsc relomegadai

% figure()
% plot(t,norm_r)
% hold on
% goodplot
% plot(t1,norm_y1,'Linewidth',2)
% plot(t2,norm_y2,'Linewidth',2)
% plot(t3,norm_y3,'Linewidth',2)
% xlabel('Time (secs)')
% ylabel('Distance (m)')
% grid on

% err = norm_y3 - norm_r; 
% plot(err)
% hold on
% goodplot
if Switch.dq_rel
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

if Switch.err
    figure()
    hold on
    goodplot()
    plot(t,y(:,2),'Linewidth',2)
    plot(t,y(:,3),'Linewidth',2)
    plot(t,y(:,4),'Linewidth',2)
    plot(tspan,y1(:,2),'Linewidth',2)
    plot(tspan,y1(:,3),'Linewidth',2)
    plot(tspan,y1(:,4),'Linewidth',2)
    xlabel('Time (s)');
    ylabel('Position Components');
    legend('r_{n1}','r_{n2}','r_{n3}','r_{l1}','r_{l2}','r_{l3}','Location','northeastoutside');
    title('r^{I} (U_l)');
    grid on
    print -depsc err_p

    figure()
    hold on
    goodplot()
    plot(t,y(:,5),'Linewidth',2)
    plot(t,y(:,6),'Linewidth',2)
    plot(t,y(:,7),'Linewidth',2)
    plot(tspan,y1(:,5),'Linewidth',2)
    plot(tspan,y1(:,6),'Linewidth',2)
    plot(tspan,y1(:,7),'Linewidth',2)
    xlabel('Time (s)');
    ylabel('Velocity Components');
    legend('v_{n1}','v_{n2}','v_{n3}','v_{l1}','v_{l2}','v_{l3}','Location','northeastoutside');
    title('v^{I}_{B/I} (U_l/U_t)');
    grid on
    print -depsc err_v
end
end