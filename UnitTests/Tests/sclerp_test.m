%%ScLERP test
%Define the initial DQ
qi = Eul2Q([pi/2,pi/4,pi/2],'XYZ');
angi = 2*rad2deg(acos(qi(4)));
ri = [2;3;4];  %translation vector
ri_norm = norm(ri);
dqi = Q2DQ(qi,ri,1); 

%Define the final DQ
qf = Eul2Q([pi/4,pi/4,pi/4],'XYZ');
angf = 2*rad2deg(acos(qf(4)));
rf = [10;-5;3];  %translation vector
rf_norm = norm(rf);
dqf = Q2DQ(qf,rf,1); 

%Define the time vector
tt = linspace(0,5,30);
T = tt./tt(end);

%ScLERP operation
[dqm,rtt,r_norm,ang,l,m,d,theta,t] = ScLERP(dqi, dqf, tt);
a = dqm(1:4,:);

rrel = rf-ri;
r_vec = (rf-ri)./norm(rf-ri);
c = norm(rf-ri);
cc = linspace(0,c,30);

for i = 1:30
    r(1:3,i) = cc(i).*r_vec;
    r_act(1:3,i) = r(1:3,i) + ri;
    dq_act(1:8,i) = Q2DQ(a(1:4,i),r_act(1:3,i),1);
end

% for i = 1:30
%     check1(1:4,i) =cross_quat(dqi(5:8),dqt(1:4,i));
%     check2(1:4,i) = dq(5:8,i)- check1(1:4,i); 
%     check3(1:4,i) = cross_quat(conj_quat(dqi(1:4)),check2(1:4,i));
%     check4(i) = sin(T(i)*ang(i)/2);
%     check5(1:3,i) = l.*(T(i)*cos(T(i)*ang(i)/2)/2);
% %     check5(1:3,i) = check4(1:3,i)./norm(m);
% %     check6(1:3,i) = check5(1:3,i).*T(i);
% end
% conv = check3-dqt(5:8,:);



% figure()
% axis equal
% grid on         
% plot(tt,ang);
% xlabel('Time (sec)');
% ylabel('Rotation (deg)');
% 
% figure()
% axis equal
% grid on         
% plot(tt,r_norm);
% xlabel('Time (sec)');
% ylabel('Position (m)');
% 
% % figure()
% % axis equal
% % grid on
% % plot3(r(1,1:30),r(2,1:30),r(3,1:30));          
% % xlabel('Time (sec)');
% % ylabel('Position (m)');
% % 
figure()
axis equal
grid on
quiver3(rtt(1,1),rtt(2,1),rtt(3,1),10*l(1),10*l(2),10*l(3));     
hold on
% quiver3(r(1,1),r(2,1),r(3,1),r(1,30),r(2,30),r(3,30)); 
plot3(rtt(1,1:30),rtt(2,1:30),rtt(3,1:30));   
plot3(r_act(1,1:30),r_act(2,1:30),r_act(3,1:30));   
xlabel('Time (sec)');
ylabel('Position (m)');
