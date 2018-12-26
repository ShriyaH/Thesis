%%Motion Planning
% for i = 1:length(n_c)
%[Traj] = Propagate_samples(n_v,Samples.D_sph_init(:,:,1),Imp_Ellipse,Kleopatra);  % Sample propagator
% end
% % 
% [Orient] = Orient_poly(Kleopatra,SC,Traj,Sun);

% [Scores] = Score_traj(Kleopatra,Sun,SC,Traj);  %Score each trajectory


%% Transform reference frame from ACFI frame to ACFR frame
q_AI = [0;0;sin(w_AI(3)*T/2);cos(w_AI(3)*T/2)];
C_AI = Q2DCM(q_AI);
rs_B = C_BA*C_AI*Sun.rs_I;
e_B = rs_B./norm(rs_B);

%% Convert to mex
codegen -config:mex T_GG -args {zeros(80,3,'double'),zeros(4,1,'double'),zeros(4,1,'double'),zeros(1,1,'double'), zeros(1,1,'double'), zeros(8,3,'double'), zeros(12,3,'double'), zeros(18,4,'double'),zeros(3,3,12,'double'), zeros(3,3,18,'double')}
codegen -config:mex orb_int_dq -args {zeros(1,1,'double'),zeros(16,1,'double')}

codegen -config:mex T_GG -args {zeros(80,3,'double'),zeros(4,1,'double'),zeros(4,1,'double'),zeros(1,1,'double'), zeros(1,1,'double'), zeros(8,3,'double'), zeros(12,3,'double'), zeros(18,4,'double'),zeros(3,3,12,'double'), zeros(3,3,18,'double')}

%% Linearisation simple example
x = -3:0.1:3;
y1 = 2.*x.^4;
y2 = 8.*x-6;
y3 = 12.*x.^2-16.*x+6;

figure()
goodplot
plot(x,y1,'Linewidth',2)
grid on
hold on
plot(x,y2,'Linewidth',2)
% plot(x,y3,'Linewidth',2)
xlabel('x-axis')
ylabel('Function Values')
legend('f_1','f_2')
