%% Relative Dynamics and Kinematics %%

x_A = (C_AI*x_I')';
v_A = (C_AI*v_I')' - (W_AI1*x_I')';
q_BA = [0 0 0 1];
C_BA = Q2DCM(q_BA);
w_AI_B = (C_BA*w_AI')';
w_BA = w_BI-w_AI_B;

[t1,y1] = test_rel_eom(T_D,I,mu,w_AI,W_AI1,x_A,v_A,w_BA,q_BA);

for j= 1:length(y1)
    norm_y1(j) = norm(y1(j,1:3));
end

%% Relative Dynamics and Kinematics %%

v_B = (C_BA * v_A')';

[t2,y2,r_B1] = test_rel_b_eom(T_D,I,mu,w_AI,x_A,v_B,w_BA,q_BA);

for j= 1:length(y2)
    norm_y2(j) = norm(y2(j,1:3));
%     q_d(j,:) = 0.5*(cross_quat([r_B(j,:) 0]',y2(j,10:13)'))';
end