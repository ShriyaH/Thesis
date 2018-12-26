clear all
close all
% state=zeros(100);
% LT=zeros(1);
global km
G=6.67408e-11;
M_ros=1e13;
mu=G*M_ros;

km=1;
cspice_furnsh( 'SPICE/standard_R.txt' )


epoch1    = 'October 15, 2014 12:00 PM UTC';
epoch2    = 'October 19, 2014 04:00 AM UTC';
epoch3    = 'July 04, 2003 11:00 AM UTC';

et1 = cspice_str2et( epoch1 )
et2 = cspice_str2et( epoch2 );
et3 = cspice_str2et( epoch3 );


ett=et1:10:et2;%+5e5;
time=ett-et1;
[a,b,c,d]=cspice_frinfo (-1000012000);
[state, lt] = cspice_spkezr('SUN', et1,'J2000', 'NONE', '67P/C-G');
% [state1, lt] = cspice_spkezr('SUN', ett,'ROS_SPACECRAFT', 'NONE', 'ROSETTA');
RS=state(1:3,:);
% state=state2-state1;
%  [state1, lt] = cspice_spkezr('1000012', ett,'J2000', 'LT', 'SUN');
%  [state3, lt] = cspice_spkezr('MARS', ut3,'J2000', 'LT+S', 'EARTH');
%  [SE, lt1] = cspice_spkezr('EARTH', et1,'J2000', 'NONE', 'SUN');
% state=state2-state1;
% 
% radii = cspice_bodvrd( '1000012', 'RADII', 3 )
% S= cspice_pxform('J2000' , 'ROS_SPACECRAFT', ett );
% R_A=state(1:3,:);
% V_A=state(4:6,:);
% for j=1:1:size(R_A,2)
% nR(j)=norm(R_A(:,j));
% nV(j)=norm(V_A(:,j));
% 
% end
% for i=1:1:size(S,3)
% % R_I(:,i)=S(:,:,i)*R_A(:,i);
% R_test(:,i)=S(:,:,i)'*[0 0 1]';
% end
% DCM=S;
% Qw=dcm2quat(DCM);
% Q_r=[Qw(:,2:4) Qw(:,1)];
% qdx=diff(Q_r(:,1));
% qdy=diff(Q_r(:,2));
% qdz=diff(Q_r(:,3));
% qds=diff(Q_r(:,4));
% 
% for i=1:1:size(Q_r,1)-1
%    qdot=[qdx(i)/10;
%          qdy(i)/10;
%          qdz(i)/10;
%          qds(i)/10;];
%      KSI=GetKSI(Q_r(i,:));
%     w(:,i)=2*KSI'*qdot;
%     
% end
% % NRI(i)=norm(R_I(:,i));
% % NRA(i)=norm(R_A(:,i));
% % 
% % N_r(i)=norm(V_A(:,i));
% % N_t(i)=sqrt(mu/norm(R_A(i))/1000);
% end
cspice_unload('standard_R.txt')
% 

% % for i=1:1:size(R_A,2)
% % R_I(:,i)=C(:,:,i)*R_A(:,i);
% % end
% Time=ett-et1;
 %% Plotting
fsz=11;
ppsz=[8 8]; 
mrg=0.1;

% figure()
% plot3(R_test(1,:),R_test(2,:),R_test(3,:));
% xlabel('X axis')
% ylabel('Y axis')
% zlabel('Z axis')


% goodplot(ppsz,mrg,fsz)
% figure()
% plot(time(1:end-1), w(1,:), time(1:end-1), w(2,:), time(1:end-1), w(3,:));
% legend wx wy wz
figure()
plot(time,RS(1,:),time,RS(2,:),time,RS(3,:));
legend x y z
goodplot(ppsz,mrg,fsz)
% print -dpdf SUNV_R.pdf
% 
% figure()
% plot3(R_I(1,:),R_I(2,:),R_I(3,:));
% figure()
% plot(Time, N_r, Time, N_t);
% legend vr vt
% 
% figure
% 
% plot(Time, V_I(1,:));
% hold on
% plot(Time, V_I(2,:));
% hold on
% plot(Time, V_I(3,:));
% 
% 
% xlabel('X axis')
% ylabel('Y axis')
% zlabel('Z axis')
% axis equal
% % 
% % 
% % [v,f]=Find_index('itokawa.txt');
% % 
% % Data=importdata('itokawa.txt');
% % V=Data.data(1:v,:);
% % F=Data.data(v+1:f,:);
% % 
% % plot3(V(:,1),V(:,2),V(:,3),'.','Color','k');
% % axis equal
% % for i=1:size(F,1)
% % 
% % line([V(F(i,1),1) V(F(i,2),1)],[V(F(i,1),2) V(F(i,2),2)],[V(F(i,1),3) V(F(i,2),3)], 'Color', 'k')
% % hold on
% % line([V(F(i,2),1) V(F(i,3),1)],[V(F(i,2),2) V(F(i,3),2)],[V(F(i,2),3) V(F(i,3),3)], 'Color', 'k')
% % hold on
% % line([V(F(i,1),1) V(F(i,3),1)],[V(F(i,1),2) V(F(i,3),2)],[V(F(i,1),3) V(F(i,3),3)], 'Color', 'k')
% % hold on
% % 
% % end
% 
% 
% goodplot(ppsz,mrg,fsz)
% print -dpdf ROSA.pdf