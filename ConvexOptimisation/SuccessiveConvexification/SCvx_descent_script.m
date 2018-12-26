%%----Ast_descent_script----%%
% clear all; clc; close all; 

global ITR CONSTANTS;
Initialize_models;

[tc,xc,uc,xdot,cpu_ps] = SCvx_transcription();


% postprocess output
t = ITR.t_k;
x = ITR.x_k{end};
u = ITR.u_k{end};
xdot = ITR.xdot_k{end};
m = x(1,:);

% n = 6;
% Tmax = 3100;
% phi = 27;
% 
% 
% Tc = [u(1,:).*m;
%     u(2,:).*m;
%     u(3,:).*m];
% 
% 
% figure;
% plot(t,u(1:3,:))
% hold on;
% 
% 
% n = 6;
% Tmax = 3100;
% phi = 27;
% T(1,:) = .001*u(1,:).*m;
% T(2,:) = .001*u(2,:).*m;
% T(3,:) = .001*u(3,:).*m;
% T_N(1,:) = u(1,:).*m;
% T_N(2,:) = u(2,:).*m;
% T_N(3,:) = u(3,:).*m;
% 
% G_N = u(4,:).*m;
% 
% Gamma =  .001*u(4,:).*m;
% Tc1 = u(1,:);
% Tc2 = u(2,:);
% Tc3 = u(2,:);
% 
% Tc_mag = zeros(1,size(T,2));
% for ii = 1:size(T,2)
%     Tc_mag(ii) = 1000*norm(T(:,ii));
%     T_N_mag(ii)= norm(T_N(:,ii));
% end
% Tc_mag = Tc_mag/(Tmax*n*cos(phi*pi/180));
% 
% theta_c = atan2(u(1,:),u(3,:));
% 
% % acikmese-like plots
% figure(31);
% subplot(3,2,1);
% plot(t,x(1,:),'k');
% hold on; grid on;
% plot(t,x(2,:),'k');
% plot(t,x(3,:),'k');
% wxlabel('Time (s)',14);
% wylabel('Position (m)',14);
% 
% set(gca,'FontSize',14);
% 
% subplot(3,2,2);
% plot(t,x(4,:),'k');
% hold on; grid on;
% plot(t,x(5,:),'k');
% plot(t,x(6,:),'k');
% 
% wxlabel('Time (s)',14);
% wylabel('Velocity (m/s)',14);
% set(gca,'FontSize',14);
% 
% subplot(3,2,3);
% plot(t,xdot(4,:),'k');
% hold on; grid on;
% plot(t,xdot(5,:),'k');
% plot(t,xdot(6,:),'k');
% 
% wxlabel('Time (s)',14);
% wylabel('Acceleration (m/s^2)',14);
% set(gca,'FontSize',14);
% 
% subplot(3,2,4);
% plot(t,T(1,:),'k');
% hold on; grid on;
% plot(t,T(2,:),'k');
% plot(t,T(3,:),'k');
% wxlabel('Time (s)',14);
% wylabel('Net force (kN)',14);
% set(gca,'FontSize',14);
% 
% subplot(3,2,5);
% plot(t,Tc_mag,'k');
% hold on; grid on;
% wxlabel('Time (s)',14);
% wylabel('Throttle level',14);
% set(gca,'FontSize',14);
% 
% subplot(3,2,6);
% plot(t,180/pi*abs(theta_c),'k');
% hold on; grid on;
% wxlabel('Time (s)',14);
% wylabel('\theta (deg)',14);
% set(gca,'FontSize',14);
% 
% 
% for ii = 1:6
%     subplot(3,2,ii);
%     set(gca,'XLim',[0 81]);
% end
% subplot(3,2,5);
% set(gca,'YLim',[0.25 0.85]);
% set(gcf,'Position',[1 9        1039        1108]);
% % axis auto;
% 
% figure(32);
% plot(x(1,:),x(3,:),'k');
% hold on
% plot(x(1,:),atan(CONSTANTS.theta_limit)*x(1,:),'k--')
% wylabel('z (m)',14);
% wxlabel('x (m)',14);
% axis equal;

m_spent = ITR.x_k{end}(1,1)-ITR.x_k{end}(1,end);
disp(['mass spent: ',num2str(m_spent),' kg'])

