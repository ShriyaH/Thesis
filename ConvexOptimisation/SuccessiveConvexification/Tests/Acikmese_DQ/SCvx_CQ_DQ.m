function [] = SCvx_CQ_DQ(i)
%% Compare with CQ plots
load('acik_DQ.mat')
t = ITR.t_k;
f1 = ones(1,length(t)).*CONSTANTS.F1;
f2 = ones(1,length(t)).*CONSTANTS.F2;
gm = CONSTANTS.theta_gm;
gm = ones(1,length(t)).*rad2deg(gm);
gs = CONSTANTS.theta_gs;
gs = ones(1,length(t)).*rad2deg(gs);
ta = CONSTANTS.theta_tilt;
ta = ones(1,length(t)).*rad2deg(ta);
wm = CONSTANTS.w_max;
wm = ones(1,length(t)).*rad2deg(wm);

dq_form=2;

for ii = 1:i
    for k = 1:length(t)
        x = ITR.x_k{ii}; 
        dq = norm_dq(x(2:9,k));
        C = Q2DCM(conj_quat(dq(1:4)));
        
        %get all positions
        rr(4*ii-3:4*ii,k) = DQ2R(dq,dq_form);
        
        %get torque norm
        F(ii,k) = norm(x(18:20,k));
        d(k) = x(18,k)/F(ii,k);
        
        %get gimbal angle
        if x(19,k) < 0
            Gm(ii,k) = rad2deg(acos(d(k)));
        else
            Gm(ii,k) = -rad2deg(acos(d(k)));
        end
        
        %get tilt angle
        Ta_norm(k) = 1-2*(norm(x(3:4,k)))^2; 
        vec1 = conj_quat(dq(1:4));
        if vec1(3) < 0 
            Ta(ii,k) = -rad2deg(acos(Ta_norm(k))); 
        else
            Ta(ii,k) = rad2deg(acos(Ta_norm(k)));
        end
        
        %get glide-slope angle
        Gs(ii,k) = rad2deg(atan(rr(4*ii-3,k)/norm(rr(4*ii-2:4*ii-1,k))));
        
        %get angular rate
        vec2(3*ii-2:3*ii,k) = x(10:12,k);
        if vec2(3*ii,k) < 0 
            w(ii,k) = -rad2deg(norm(x(10:12,k)));
        else
            w(ii,k) = rad2deg(norm(x(10:12,k)));
        end
        
        if ii < i
        %quat normalisation
        q_norm(ii,k) = norm(ITR.x_k{ii}(2:5,1));
        end    
               
    end
    diff(ii) = abs(ITR.x_k{end}(1,end) - ITR.x_k{ii}(1,end));
    
    %get trust region
    eta = ITR.eta_k{ii};
    eta_n(ii) = abs(mean(eta)); 
        
    %get dynamic relaxation
    v = ITR.v_k{ii}(1:33,:);
    v_t = mean(v);
    v_n(ii) = abs(mean(v_t));
    
    %state convergence and virtual controls convergence
    if ii < i
        state_conv(ii) = ITR.state_conv{ii};
        virt_cont(ii) = ITR.virt_cont{ii};
    end
end
m_spent_dq = ITR.m_spent;

load('acik_CQ.mat')
for ii = 1:i
    for k = 1:30
        x1 = ITR.x_k{ii}; 
        q1 = x1(8:11,k)./norm(x1(8:11,k));
        C1 = Q2DCM(conj_quat(q1));
        %get torque norm
        T1(ii,k) = norm(x1(15:17,k));
        d1(k) = x1(15,k)/T1(ii,k);
        
        %get gimbal angle
        if x1(16,k) < 0
            Gm1(ii,k) = -rad2deg(acos(d1(k)));
        else
            Gm1(ii,k) = rad2deg(acos(d1(k)));
        end
        
        %get tilt angle
        Ta_norm1(k) = 1-2*(norm(x1(9:10,k)))^2; 
        vec11 = conj_quat(q1);
        if vec11(3) < 0 
            Ta1(ii,k) = -rad2deg(acos(Ta_norm1(k))); 
        else
            Ta1(ii,k) = rad2deg(acos(Ta_norm1(k)));
        end
        
        %get all positions
        r1(3*ii-2:3*ii,k) = x1(2:4,k); 
        
        %get glide-slope angle
        Gs1(ii,k) = rad2deg(atan(x1(2,k)/norm(x1(3:4,k))));
        
        %get angular rate
        vec21(3*ii-2:3*ii,k) = x1(12:14,k);
        if vec21(3*ii,k) < 0 
            w1(ii,k) = -rad2deg(norm(x1(12:14,k)));
        else
            w1(ii,k) = rad2deg(norm(x1(12:14,k)));
        end
        
        if ii < i
        %quat normalisation
        q_norm1(ii,k) = norm(ITR.x_k{ii}(8:11,1));
        end    
               
    end
    diff1(ii) = abs(ITR.x_k{end}(1,end) - ITR.x_k{ii}(1,end));
    
    %get trust region
    eta1 = ITR.eta_k{ii};
    eta_n1(ii) = abs(mean(eta1)); 
        
    %get dynamic relaxation
    v1 = ITR.v_k{ii}(1:20,:);
    v_t1 = mean(v1);
    v_n1(ii) = abs(mean(v_t1));
    
    %state convergence and virtual controls convergence
    if ii < i
        state_conv1(ii) = ITR.state_conv{ii};
        virt_cont1(ii) = ITR.virt_cont{ii};
    end
end
m_spent_cq = ITR.m_spent;

%plot consumed mass
figure()
plot(1:i,m_spent_dq(1:i),'Marker','.','Color','b')
hold on
plot(1:i,m_spent_cq(1:i),'Marker','.','Color','r')
axis([1 10 0.8 1.3])
xlabel('Iterations')
ylabel('Mass Spent')
legend('SCvx_{DQ}','SCvx_{CQ}','Location','northeastoutside')
grid on

%plot difference in converged end mass
figure()
plot(1:i-1,diff(1:i-1),'Marker','.','Color','b')
hold on
plot(1:i-1,diff1(1:i-1),'Marker','.','Color','r')
xlabel('Iterations')
ylabel('Difference in Mass Spent')
set(gca, 'YScale', 'log')
axis([1 i-1 10e-20 10e0])
legend('SCvx_{DQ}','SCvx_{CQ}','Location','northeastoutside')
grid on
hold off

%plot difference in converged state
figure()
axis tight
plot(1:i-1,state_conv(1:i-1),'Marker','.','Color','b')
hold on
plot(1:i-1,state_conv1(1:i-1),'Marker','.','Color','r')
axis([1 10 0 100])
xlabel('Iterations')
ylabel('Difference in Converged State')
legend('SCvx_{DQ}','SCvx_{CQ}','Location','northeastoutside')
grid on

%plot difference in converged virtual controls
figure()
axis tight
plot(1:i-1,virt_cont(1:i-1),'Marker','.','Color','b')
hold on
plot(1:i-1,virt_cont1(1:i-1),'Marker','.','Color','r')
axis([1 10 0 0.6])
xlabel('Iterations')
ylabel('Virtual Controls Norm')
legend('SCvx_{DQ}','SCvx_{CQ}','Location','northeastoutside')
grid on

%trust region
figure()
plot(0:i-1,eta_n(1:i),'Marker','.','Color','b')
hold on
plot(0:i-1,eta_n1(1:i),'Marker','.', 'Color', 'r')
xlabel('Iterations')
ylabel('Trust Region')
set(gca, 'YScale', 'log')
axis([1 10 10e-10 10e0])
legend('SCvx_{DQ}','SCvx_{CQ}','Location','northeastoutside')
grid on

%dynamic relaxation
figure()
plot(0:i-1,v_n(1:i),'Marker','.','Color','b')
hold on
plot(0:i-1,v_n1(1:i),'Marker','.', 'Color', 'r')
xlabel('Iterations')
ylabel('Dynamic Relaxation')
set(gca, 'YScale', 'log')
axis([1 10 10e-10 10e0])
legend('SCvx_{DQ}','SCvx_{CQ}','Location','northeastoutside')
grid on


%plot thrusts
figure()
plot(t,f1,'LineWidth',2, 'Color', 'red');
hold on
plot(t,f2,'LineWidth',2, 'Color', 'red');
plot(t,F(11,:),'Marker','.', 'Color', 'b')
plot(t,T1(11,:),'Marker','.', 'Color', 'r')
xlabel('Time (s)');
ylabel('Thrust (N)');
legend('T_{max}','T_{min}','SCvx_{DQ}','SCvx_{CQ}','Location','northeastoutside')
grid on

%plot gimbal angles
figure()
plot(t,gm,'LineWidth',2, 'Color', 'red');
hold on
plot(t,-gm,'LineWidth',2, 'Color', 'red');
plot(t,Gm(11,:),'Marker','.','Color', 'b');
plot(t,Gm1(11,:),'Marker','.','Color', 'r');
xlabel('Time (s)');
ylabel('Gimbal Angle (Deg)');
legend('\theta_{gm_{max}}','\theta_{gm_{min}}','SCvx_{DQ}','SCvx_{CQ}','Location','northeastoutside')
grid on

%plot tilt angles
figure()
plot(t,ta,'LineWidth',2, 'Color', 'red');
hold on
plot(t,-ta,'LineWidth',2, 'Color', 'red');
plot(t,Ta(11,:),'Marker','.','Color', 'b');
plot(t,Ta1(11,:),'Marker','.','Color', 'r');
xlabel('Time (s)');
ylabel('Tilt Angle (Deg)');
legend('\delta_{ta_{max}}','\delta_{ta_{min}}','SCvx_{DQ}','SCvx_{CQ}','Location','northeastoutside')
grid on

%plot angular rates
figure()
plot(t,wm,'LineWidth',2, 'Color', 'r');
hold on
plot(t,-wm,'LineWidth',2, 'Color', 'r');
plot(t,w(11,:),'Marker','.','Color', 'b');
plot(t,w1(11,:),'Marker','.','Color', 'r');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
legend('\omega_{max}','\omega_{min}','SCvx_{DQ}','SCvx_{CQ}','Location','northeastoutside')
grid on

%plot glide-slope angles
figure()
plot(t,gs,'LineWidth',2, 'Color', 'r');
hold on
plot(t,Gs(11,:),'Marker','.','Color', 'b');
plot(t,Gs1(11,:),'Marker','.','Color', 'r');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
legend('\gamma_{min}','SCvx_{DQ}','SCvx_{CQ}','Location','northeastoutside')
grid on

%plot 2D trajectories
j=11;
figure()
subplot(2,3,1)
h1 =plot(rr(4*j-2,:),rr(4*j-3,:),'Marker','.','Color', 'b');
hold on
plot(r1(3*j-1,:),r1(3*j-2,:),'Marker','.','Color', 'r');
xlabel('East (m)');
ylabel('Up (m)');
grid on

subplot(2,3,2)
plot(rr(4*j-1,:),rr(4*j-3,:),'Marker','.','Color', 'b');
hold on
plot(r1(3*j,:),r1(3*j-2,:),'Marker','.','Color', 'r');
xlabel('North (m)');
ylabel('Up (m)');
legend('SCvx_{DQ}','SCvx_{CQ}','Location','northeastoutside')
grid on

subplot(2,3,[4 5])
plot(rr(4*j-2,:),rr(4*j-1,:),'Marker','.','Color', 'b');
hold on 
plot(r1(3*j-1,:),r1(3*j,:),'Marker','.','Color', 'r');
xlabel('East (m)');
ylabel('North (m)');
grid on

% Legend = ('SCvx_{DQ}';'SCvx_{CQ}');
% hL = subplot(2,3,3);
% poshL = get(hL,'position');     % Getting its position
% 
% lgd = legend(hL, h1,Legend);
% set(lgd,'position',poshL);      % Adjusting legend's position
% axis(hL,'off');   

%plot 3D trajectories 2
figure()
plot3(rr(4*j-2,:),rr(4*j-1,:),rr(4*j-3,:));
hold on
plot3(r1(3*j-1,:),r1(3*j,:),r1(3*j-2,:));
xlabel('East (m)');
ylabel('North (m)');
zlabel('Up (m)');
legend('SCvx_{DQ}','SCvx_{CQ}','Location','northeastoutside')
grid on

% %plot 3D trajectories
% figure()
% comet3(rr(4*j-2,:),rr(4*j-1,:),rr(4*j-3,:));
% hold on
% comet3(r1(3*j-1,:),r1(3*j,:),r1(3*j-2,:));
% xlabel('East (m)');
% ylabel('North (m)');
% zlabel('Up (m)');
% legend('SCvx_{DQ}','SCvx_{CQ}','Location','northeastoutside')
% grid on
end