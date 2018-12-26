function [T_GG_T] = T_GG(vv,vert_pm,q_BI,q_AI,m_n,rho,V,F,E,F_tilde,E_tilde)

%Number of point masses: 80(20 block, 60 solar arrays)
n_block = 8 + 12; %8 vertices, 12 facet centres
n_SA = 40 + 20; % 4 vertices and 2 facet centres each on 10 panels

%Mass of point masses
m = [((m_n-150)/n_block).*(ones(n_block,1)); (150/n_SA).*(ones(n_SA,1))]; %mass of SA: 150kg 

%Position of point masses wrt inertial frame
v_I = vert_pm;

%Initialise
v_A = zeros(length(v_I),3);
g_A = zeros(3,length(v_I));
g_I = zeros(3,length(v_I));
g_B = zeros(3,length(v_I));
T_GG = zeros(length(v_I),3);

%Grav accl, gradient torque  of point masses wrt body frame
for i = 1:length(v_I)
    v_A(i,:) = quat_trans(q_AI,[v_I(i,:)';0],'vect');
    

    [g_A(:,i),g_I(:,i),g_B(:,i),~] = Poly_g(v_A(i,:)',q_BI,q_AI,rho,V,F,E,F_tilde,E_tilde);
    
    F_B(i,:) = (m(i,:) * g_B(1:3,i))';
    
    T_GG(i,1:3) = cross(vv(i,:),F_B(i,:),2);
   
end
T_GG_T = sum(T_GG(:,1:3))';
end



