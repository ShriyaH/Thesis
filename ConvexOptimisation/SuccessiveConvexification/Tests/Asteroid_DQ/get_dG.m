function [F_GG_Bc,T_GG_Bc,dgb] = get_dG(m,dq)
global Kleopatra SC CONSTANTS
C = SC.Polyhedron.C;
V = SC.Polyhedron.Vertices;
dq_form = CONSTANTS.dq_form;
% dga =[];
% % dga2 = [];
% dgb =[];
% dgb2=[];
count = 1;
%     for h = 10.^(-1:16)
%         count=count+1;
h = 10.^16;

    % complex differentiation
    dqc(1:8,count) = dq + 1i*h;
    dqc(1:8,count) = norm_dq(dqc(1:8,count));

    r_Ac = 2*cross_quat(dqc(5:8,count),conj_quat(dqc(1:4,count)));
    r_Ac = r_Ac(1:3);
    q_BAc = dqc(1:4,count);
    C_BAc = Q2DCM(q_BAc);
    r_Bc = quat_trans(q_BAc,r_Ac,'vect');

    [g_Ac, g_Bc, Wfc, Uc] = Poly_g_new(r_Ac, q_BAc,Kleopatra);

    dga(1:3,count) = imag(g_Ac)./h;
    dgb(1:4,count) = imag(g_Bc)./h;


    n_block = 8 + 12; %8 vertices, 12 facet centres
    n_SA = 40 + 20; % 4 vertices and 2 facet centres each on 10 panels

    %Mass of point masses
    m_pm = [((m-150)/n_block).*(ones(n_block,1)); (150/n_SA).*(ones(n_SA,1))]; %mass of SA: 150kg 

    %Position of point masses wrt body frame
    r_pm = [ V(1:8,:); C(1:12,:); V(9:48,:); C(13:32,:) ];
    r_pm_Bc = -r_pm + r_Bc;

    %Grav accl, gradient torque  of point masses wrt body frame
    for i = 1:length(m_pm)
        r_pm_Ac(:,i) = C_BAc'*r_pm_Bc(i,:)';
        [g_pm_Ac(:,i),g_pm_Bc(:,i)] = Poly_g_new(r_pm_Ac(:,i),q_BAc,Kleopatra);
%             dga_pm(:,i) = imag(g_pm_Ac(:,i))./h;
%             dgb_pm(:,i) = imag(g_pm_Bc(:,i))./h;

        F_GGc(i,:) = (m_pm(i) .* g_pm_Bc(1:3,i)');
        T_GGc(i,:) = cross(r_pm(i,:),F_GGc(i,:),2);
    end
    F = sum(F_GGc)';
    T = sum(T_GGc)';
    F_GG_Bc = [imag(F)./h;0];
    T_GG_Bc = [imag(T)./h;0];
end
