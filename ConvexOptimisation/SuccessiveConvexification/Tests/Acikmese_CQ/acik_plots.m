function [] = acik_plots(i)
global CONSTANTS ITR

t = ITR.t_k;
T1 = ones(1,length(t)).*CONSTANTS.T1;
T2 = ones(1,length(t)).*CONSTANTS.T2;
gm = CONSTANTS.theta_gm;
gm = ones(1,length(t)).*rad2deg(gm);
gs = CONSTANTS.theta_gs;
gs = ones(1,length(t)).*rad2deg(gs);
ta = CONSTANTS.theta_tilt;
ta = ones(1,length(t)).*rad2deg(ta);
wm = CONSTANTS.w_max;
wm = ones(1,length(t)).*rad2deg(wm);

Legend = cell(i+2,1);

for ii = 1:i
    for k = 1:30
        x = ITR.x_k{ii}; 
        q = x(8:11,k)./norm(x(8:11,k));
        C = Q2DCM(conj_quat(q));
        %get torque norm
        T(ii,k) = norm(x(15:17,k));
        d(k) = x(15,k)/T(ii,k);
        
        %get gimbal angle
        if x(16,k) < 0
            Gm(ii,k) = -rad2deg(acos(d(k)));
        else
            Gm(ii,k) = rad2deg(acos(d(k)));
        end
        
        %get tilt angle
        Ta_norm(k) = 1-2*(norm(x(9:10,k)))^2; 
        vec1 = conj_quat(q);
        if vec1(3) < 0 
            Ta(ii,k) = -rad2deg(acos(Ta_norm(k))); 
        else
            Ta(ii,k) = rad2deg(acos(Ta_norm(k)));
        end
        
        %get all positions
        r(3*ii-2:3*ii,k) = x(2:4,k); 
        
        %get glide-slope angle
        Gs(ii,k) = rad2deg(atan(x(2,k)/norm(x(3:4,k))));
        
        %get angular rate
        vec2(3*ii-2:3*ii,k) = C*x(12:14,k);
        if vec2(3*ii,k) < 0 
            w(ii,k) = -rad2deg(norm(x(12:14,k)));
        else
            w(ii,k) = rad2deg(norm(x(12:14,k)));
        end
        
        if ii < i
        %quat normalisation
        q_norm(ii,k) = norm(ITR.x_k{ii}(8:11,1));
        end    
               
    end
    diff(ii) = abs(ITR.x_k{end}(1,end) - ITR.x_k{ii}(1,end));
    
    %get trust region
    eta = ITR.eta_k{ii};
    eta_n(ii) = abs(mean(eta)); 
        
    %get dynamic relaxation
    v = ITR.v_k{ii}(1:20,:);
    v_t = mean(v);
    v_n(ii) = abs(mean(v_t));
    
    %state convergence and virtual controls convergence
    if ii < i
        state_conv(ii) = ITR.state_conv{ii};
        virt_cont(ii) = ITR.virt_cont{ii};
    end
end


%plot consumed mass
figure()
plot(1:i,ITR.m_spent(1:i),'Marker','o')
xlabel('Iterations')
ylabel('Mass Spent')
grid on

%plot difference in converged end mass
figure()
plot(1:i-1,diff(1:i-1),'Marker','.','Color','b')
hold on
xlabel('Iterations')
ylabel('Difference in Mass Spent')
set(gca, 'YScale', 'log')
axis([1 i-1 10e-10 10e0])
grid on

%plot difference in converged state
figure()
plot(1:i-1,state_conv(1:i-1),'Marker','o')
xlabel('Iterations')
ylabel('Difference in Converged State')
% set(gca, 'YScale', 'log')
% axis([1 i-1 10e-10 10e0])
grid on

%plot difference in converged state
figure()
plot(1:i-1,virt_cont(1:i-1),'Marker','o')
xlabel('Iterations')
ylabel('Virtual Controls')
% set(gca, 'YScale', 'log')
% axis([1 i-1 10e-10 10e0])
grid on

%plot thrusts
figure()
plot(t,T1,'LineWidth',2, 'Color', 'red');
hold on
plot(t,T2,'LineWidth',2, 'Color', 'red');
Legend{1}= strcat('T_{max}');
Legend{2}= strcat('T_{min}');
for j = 1:i 
    plot(t,T(j,:),'Marker','.')
    Legend{j+2}=strcat('Itr', num2str(j));
end
xlabel('Time (s)');
ylabel('Thrust (N)');
legend(Legend,'Location','northeastoutside')
grid on

%plot gimbal angles
figure()
plot(t,gm,'LineWidth',2, 'Color', 'red');
hold on
plot(t,-gm,'LineWidth',2, 'Color', 'red');
Legend{1}= strcat('\theta_{gm_{max}}');
Legend{2}= strcat('\theta_{gm_{min}}');
for j = 1:i 
    plot(t,Gm(j,:),'Marker','.');
end
xlabel('Time (s)');
ylabel('Gimbal Angle (Deg)');
legend(Legend,'Location','northeastoutside');
grid on

%plot tilt angles
figure()
plot(t,ta,'LineWidth',2, 'Color', 'red');
hold on
plot(t,-ta,'LineWidth',2, 'Color', 'red');
Legend{1}= strcat('\delta_{ta_{max}}');
Legend{2}= strcat('\delta_{ta_{min}}');
for j = 1:i 
    plot(t,Ta(j,:),'Marker','.');
end
xlabel('Time (s)');
ylabel('Tilt Angle (Deg)');
legend(Legend,'Location','northeastoutside');
grid on

%plot angular rates
figure()
plot(t,wm,'LineWidth',2, 'Color', 'red');
hold on
plot(t,-wm,'LineWidth',2, 'Color', 'red');
Legend{1}= strcat('\omega_{max}');
Legend{2}= strcat('\omega_{min}');
for j = 1:i
    plot(t,w(j,:),'Marker','.');
end
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
legend(Legend,'Location','northeastoutside');
grid on

%plot glide slope angles
Legend = cell(i+1,1);
figure()
plot(t,gs,'LineWidth',2, 'Color', 'red');
hold on
Legend{1}= strcat('\gamma_{gs_{min}}');
for j = 1:i 
    plot(t,Gs(j,:),'Marker','.');
    Legend{j+1}=strcat('Itr', num2str(j));
end
xlabel('Time (s)');
ylabel('Glide-Slope Angle (Deg)');
legend(Legend,'Location','northeastoutside');
grid on

%plot 2D trajectories
Legend = cell(i,1);
figure()
subplot(2,3,1)
hold on
for j = 1:i
	h1(j) = plot(r(3*j-1,:),r(3*j-2,:),'Marker','.');
    Legend{j}=strcat('Itr', num2str(j));
end
xlabel('East (m)');
ylabel('Up (m)');
grid on

subplot(2,3,2)
hold on
for j = 1:i
    plot(r(3*j,:),r(3*j-2,:),'Marker','.');
end
xlabel('North (m)');
ylabel('Up (m)');
grid on

subplot(2,3,[4 5])
hold on
for j = 1:i
    plot(r(3*j-1,:),r(3*j,:),'Marker','.');
end
xlabel('East (m)');
ylabel('North (m)');
grid on

hL = subplot(2,3,3);
poshL = get(hL,'position');     % Getting its position

lgd = legend(hL, h1, Legend);
set(lgd,'position',poshL);      % Adjusting legend's position
axis(hL,'off');   

%plot 3D trajectories
figure()
hold on
for j = 1:i
comet3(r(3*j-1,:),r(3*j,:),r(3*j-2,:));
end
xlabel('East (m)');
ylabel('North (m)');
zlabel('Up (m)');
grid on

%plot 3D trajectories 2
figure()
hold on
for j = 1:i
plot3(r(3*j-1,:),r(3*j,:),r(3*j-2,:));
end
xlabel('East (m)');
ylabel('North (m)');
zlabel('Up (m)');
legend(Legend,'Location','northeastoutside');
grid on

%plot quat norm
Legend = cell(i-1,1);
figure()
hold on
for j = 1:i-1 
    plot(t,q_norm(j,:),'Marker','.');
    Legend{j}=strcat('Itr', num2str(j+1));
end
xlabel('Time (s)');
ylabel('Quaternion norm');
legend(Legend,'Location','northeastoutside');
grid on

%% Compare with Acikmese plots
%plot difference in converged end mass
y = [0.3766,0.06019,0.02056,0.004083,0.002399,0.001018,...
    5.633e-6,9.8e-6,6.417e-6,4.2e-6];
figure()
plot(1:i-1,diff(1:i-1),'Marker','.','Color','b')
hold on
plot(1:10,y,'Marker','.', 'Color', 'r')
xlabel('Iterations')
ylabel('Difference in Mass Spent')
set(gca, 'YScale', 'log')
axis([1 i-1 10e-10 10e0])
legend('Converged_{sol}','Acikmese_{sol}')
grid on

%trust region
y = [3925,2372,1127,866,523,195.7,...
    4.99e-3,6.49e-5,6.34e-5,6.19e-5];
figure()
plot(0:i-1,eta_n,'Marker','.','Color','b')
hold on
plot(1:10,y,'Marker','.', 'Color', 'r')
xlabel('Iterations')
ylabel('Trust Region')
set(gca, 'YScale', 'log')
axis([0 i-1 10e-10 10e10])
legend('Converged_{sol}','Acikmese_{sol}')
grid on

%dynamic relaxation
y = [0.0509,0.00648,0.00515,0.002061,0.001037,1.075e-6,...
    1.805e-11,2.27e-11,1.805e-11,2.27e-11];
figure()
plot(0:i-1,v_n,'Marker','.','Color','b')
hold on
plot(1:10,y,'Marker','.', 'Color', 'r')
xlabel('Iterations')
ylabel('Dynamic Relaxation')
set(gca, 'YScale', 'log')
axis([0 i-1 10e-20 10e0])
legend('Converged_{sol}','Acikmese_{sol}')
grid on


%plot thrusts
figure()
y = [1.9934,2.8061,3.0155,2.9888,3.0146,3.0141,3.0006,3.0001,2.9997,2.8943,...
    2.4086,1.5557,0.8208,0.4925,0.492,0.5571,0.5174,0.5431,1.0804,2.0504,2.8238,...
    2.902,2.272,1.1962,0.54,0.6313,1.2735,1.9157,1.8365,1.0623];
plot(t,T1,'LineWidth',2, 'Color', 'red');
hold on
plot(t,T2,'LineWidth',2, 'Color', 'red');
plot(t,y,'Marker','.', 'Color', 'b');
plot(t,T(11,:),'Marker','.')
xlabel('Time (s)');
ylabel('Thrust (N)');
legend('T_{max}','T_{min}','Acikmese_{sol}','Converged_{sol}','Location','northeastoutside')
grid on

%plot gimbal angles
y = [0.1485,-9.9505,-9.9505,-7.9703,1.8317,10.0495,10.2475,6.4851,3.7129,...
    6.3861,10.2475,10.2475,10.1485,6.2871,-5.8911,-9.9505,-9.8515,-9.8515,...
    -9.9505,-9.9505,-9.9505,-9.9505,-9.8515,-8.1683,2.6238,10.2475,10.1485,10.1485,10.1485,0.0495];
figure()
plot(t,gm,'LineWidth',2, 'Color', 'red');
hold on
plot(t,-gm,'LineWidth',2, 'Color', 'red');
plot(t,y,'Marker','.', 'Color', 'b');
plot(t,Gm(11,:),'Marker','.');
xlabel('Time (s)');
ylabel('Gimbal Angle (Deg)');
legend('\theta_{gm_{max}}','\theta_{gm_{min}}','Acikmese_{sol}','Converged_{sol}','Location','northeastoutside')
grid on

%plot tilt angles
y = [-0.0002,0.401,2.185,5.942,10.685,15.626,18.593,19.785,20.186,19.601,...
    18.029,14.878,11.134,6.404,1.675,-2.661,-6.799,-10.739,-14.088,-17.239,...
    -18.614,-18.804,-17.021,-14.251,-10.297,-6.738,-3.968,-1.197,-0.007,0.395];
figure()
plot(t,ta,'LineWidth',2, 'Color', 'red');
hold on
plot(t,-ta,'LineWidth',2, 'Color', 'red');
plot(t,y,'Marker','.', 'Color', 'b');
plot(t,Ta(11,:),'Marker','.');
xlabel('Time (s)');
ylabel('Tilt Angle (Deg)');
legend('\delta_{ta_{max}}','\delta_{ta_{min}}','Acikmese_{sol}','Converged_{sol}','Location','northeastoutside')
grid on

%plot angular rates
y = [-3.35e-08,-6.053,-16.316,-26.053,-29.737,-22.895,-12.105,-3.421,1.316,...
    6.053,13.947,20.789,24.737,26.842,26.842,25,23.158,21.579,18.947,13.684,5,...
    -5,-13.947,-19.737,-21.053,-19.737,-16.579,-10.789,-3.947,-2.12e-08];
figure()
plot(t,wm,'LineWidth',2, 'Color', 'red');
hold on
plot(t,-wm,'LineWidth',2, 'Color', 'red');
plot(t,y,'Marker','.', 'Color', 'b');
plot(t,w(11,:),'Marker','.');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
legend('\omega_{max}','\omega_{min}','Acikmese_{sol}','Converged_{sol}','Location','northeastoutside')
grid on
end