%% Dynamics and Kinematics block check 
%Inertial, Relative in A, Relative in B, Relative in B with DQ, 6dof with
%and without controls
clear data
clc
global Sun SC I m mu Kleopatra Var Switch

%% Common Input Values 
Initialize_models;

% w_AI = Kleopatra.w_AI;
w_AI = [0 0 0];
W_AI1 = omega_tensor(w_AI,1);                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
W_AI2 = omega_tensor(w_AI,2);

%For dynamic block check
% x_I = [0 60e3 0];
% v_I = [53.947598031175893 0 0];
% w_BI = [6.28318e-4 0 0];

%For SRP check and 3BP check
% x_I = [0 0 60e3];
% v_I = [0 53.947598031175893 0];
% w_BI = [0 0 0];
x_I = [0.689443571879531   0.689443571879531   0.222115110670095].*100e3;
v_I = [0.316318321278693  -0.010714267062475  -0.948592601755223].*55.648657845450323;
w_BI = [0 0 0];


q_BI = [0 0 0 1];
q_AI = [0 0 0 1];

C_AI = Q2DCM(q_AI);

%% Select Integrator 
Switch.Razgus = 0;
Switch.Q = 1;
Switch.Q_rel = 0;
Switch.DQ = 0;
Switch.DOF6 = 0;
Switch.DOF6_lin = 0;

%% Plot
Switch.convert_q = 0;
Switch.kepler_el = 0;
Switch.kepler_n= 1;
Switch.err = 0;

%% Inertial Dynamics and Kinematics
if Switch.Q 
    [Var.t,Var.y] = test_inert_eom(x_I,v_I,w_BI,q_BI,q_AI,W_AI2); %integrate for inertial frame (Cartesian and Quaternions)

%Convert to relative frame 
    for i = 1:length(Var.y)
        Var.norm_ri(i) = norm(Var.y(i,1:3));
        
        q_IA(:,i) = conj_quat(Var.y(i,14:17)');
        C_AIt(3*i-2:3*i,:) = Q2DCM(Var.y(i,14:17)');

        C_BIt(3*i-2:3*i,:) = Q2DCM(Var.y(i,10:13)');

        Var.q_BAt(1:4,i) = cross_quat(Var.y(i,10:13)',q_IA(:,i));
        C_BAt(3*i-2:3*i,:) = Q2DCM(Var.q_BAt(1:4,i));

        Var.r_At(:,i) = C_AIt(3*i-2:3*i,:)*Var.y(i,1:3)';
        Var.r_Bt(:,i) = C_BAt(3*i-2:3*i,:)*Var.r_At(:,i);

        Var.v_At(:,i) = C_AIt(3*i-2:3*i,:)*Var.y(i,4:6)'- W_AI1*Var.r_At(:,i);
        Var.v_Bt(:,i) = C_BAt(3*i-2:3*i,:)*Var.v_At(:,i);

        Var.w_BA_b(:,i) = Var.y(i,7:9)'- C_BAt(3*i-2:3*i,:)*w_AI';
        Var.w_BA_a(:,i) = C_BAt(3*i-2:3*i,:)'*Var.w_BA_b(:,i);
        
        Var.w_AI_B(:,i) = C_BAt(3*i-2:3*i,:)*w_AI';
    end
end

%% Relative Dynamics and Kinematics: Dual Quaternions %%

%Relative frame Initialisation
x_A = (C_AI*x_I');
v_BA_A = (C_AI*v_I') - (W_AI1*x_A);
q_BA = cross_quat(q_BI',q_AI');
C_BA = Q2DCM(q_BA);

v_BA_B = C_BA*v_BA_A;
w_AI_B = (C_BA*w_AI');
w_BA_B = w_BI'-w_AI_B;

if Switch.Q_rel    
    [Var.t2,Var.y2,Var.r_B] = test_rel_b_eom(w_AI,x_A',v_BA_B',w_BA_B',q_BA');
    for j = 1:length(Var.y2)
        Var.norm_rb(j) = norm(Var.r_B(j,1:3));
        Var.norm_ra(j) = norm(Var.y2(j,1:3));
    end
end

if Switch.DQ 
    %change to DQ format
    dI = dq_inertia(SC.mass.m_i,SC.I.I_total);
    dw_BA = [w_BA_B' 0 v_BA_B' 0];
    dq_BA = Q2DQ(q_BA,x_A',2)';

    %warning('off','all')

    [Var.t3,Var.y3,Var.dqr_A,Var.dqr_B] = test_rel_dq_eom(dI,w_AI,dq_BA,dw_BA);
    
    for j = 1:length(Var.y3)
        Var.norm_dqra(j) = norm(Var.dqr_A(j,1:3));
        Var.norm_dqrb(j) = norm(Var.dqr_B(j,1:3));
        
        C_BAtt(3*j-2:3*j,:) = Q2DCM(Var.y3(j,1:4)');
        C_ABtt(3*j-2:3*j,:) = C_BAtt(3*j-2:3*j,:)';
        
        C_AItt(3*j-2:3*j,:) = Q2DCM([0;0;sin((w_AI(3)*Var.t3(j))/2);cos((w_AI(3)*Var.t3(j))/2)]);
        C_IAtt(3*j-2:3*j,:) = C_AItt(3*j-2:3*j,:)';
        
        Var.dqr_I(j,1:3) = (C_IAtt(3*j-2:3*j,:) * Var.dqr_A(j,1:3)')';
        Var.dqv_I(j,1:3) = (C_IAtt(3*j-2:3*j,:) * (C_ABtt(3*j-2:3*j,:)*Var.y3(j,13:15)' + (W_AI1*Var.dqr_A(j,1:3)')))';
        
        C_BAtt(3*j-2:3*j,:) = Q2DCM(Var.y3(j,1:4)');
        Var.dqw_AI_B(:,j) = C_BAtt(3*j-2:3*j,:)*w_AI';
    end
end

%% Inertial Descent: Non-linear %%
m_sc = 2;
g = [0;0;-1];
x_I = [1; 0; 2];
v_I = [0.2; 0; -1]; 
q_BI = [0; 0; 0; 1]; 
w_BI = [0; 0; 0.3]; 
Th_B = [0; 0; 2];
Th_dot_B = [0;0;0];
I = 0.5.*eye(3);
    
if Switch.DOF6    
    [t,y] = test_inert_nonlineom(I,m_sc,x_I,v_I,q_BI,w_BI,Th_B,Th_dot_B,g,q_inert);    
end

%% Inertial Descent: Linear %%

if Switch.DOF6_lin 
    [tspan,tode4,y1] = test_inert_lineom(I,m_sc,x_I,v_I,q_BI,w_BI,Th_B,Th_dot_B,g,t,y,q_inert);    
end

dyn_block_plots();