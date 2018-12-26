function [] = acik_plots3(i)
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
        if vec1(3) < 0 && vec1(1) < 0 
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
end


%plot consumed mass
figure()
plot(1:i,ITR.m_spent(1:i),'Marker','o')
xlabel('Iterations')
ylabel('Mass Spent')
grid on

%plot difference in converged end mass
figure()
plot(1:i-1,diff(1:i-1),'Marker','o')
xlabel('Iterations')
ylabel('Difference in Mass Spent')
set(gca, 'YScale', 'log')
axis([1 i-1 10e-10 10e0])
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

end