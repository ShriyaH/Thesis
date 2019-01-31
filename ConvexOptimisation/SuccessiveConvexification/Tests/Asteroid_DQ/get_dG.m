function [F_GG_Bc,T_GG_Bc,g_Bc] = get_dG(m,dq)
global Kleopatra SC
C = SC.Polyhedron.C;
V = SC.Polyhedron.Vertices;

% dga =[];
% % dga2 = [];
% dgb =[];
% dgb2=[];
count = 1;
%     for h = 10.^(-1:16)
%         count=count+1;
h = 10^-13;

        % complex differentiation
        dqc(1:8,count) = dq + 1i*h;
        dqc=norm_dq(dqc(1:8,count));

        r_Ac = 2*cross_quat(dqc(5:8),conj_quat(dqc(1:4)));
        r_Ac = r_Ac(1:3);
        q_BAc = dqc(1:4);
        r_Bc = quat_trans(dqc(1:4),r_Ac,'vect');
        C_BAc = Q2DCM(q_BAc);
        
        [g_Ac(1:3,count), g_Bc(1:4,count), Wfc(count), Uc(count)] = Poly_g_new(r_Ac, q_BAc,Kleopatra);

        g_Ac(1:3,count) = imag(g_Ac(1:3,count))./h;
        g_Bc(1:4,count) = imag(g_Bc(1:4,count))./h;
        
        
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

        F_GG_Bc = [imag(sum(F_GGc)')./h;0];
        T_GG_Bc = [imag(sum(T_GGc)')./h;0];

%         %% finite differentiation
%         dqc2(1:8,count) = dq + h;
%         dqc2=norm_dq(dqc2(1:8,count));
% 
%         dqc3(1:8,count) = dq - h;
%         dqc3=norm_dq(dqc3(1:8,count));
% 
%         r_Ac2 = 2*cross_quat(dqc2(5:8),conj_quat(dqc2(1:4)));
%         r_Ac2 = r_Ac2(1:3);
%         q_BAc2 = dqc2(1:4);
% 
%         [g_Ac2(1:3,count), g_Bc2(1:4,count), Wfc2(count), Uc2(count)] = Poly_g_new(r_Ac2, q_BAc2,Kleopatra);
% 
%         r_Ac3 = 2*cross_quat(dqc3(5:8),conj_quat(dqc3(1:4)));
%         r_Ac3 = r_Ac3(1:3);
%         q_BAc3 = dqc3(1:4);
% 
%         [g_Ac3(1:3,count), g_Bc3(1:4,count), Wfc3(count), Uc3(count)] = Poly_g_new(r_Ac3, q_BAc3,Kleopatra);
% 
%         dga2(1:3,count) = (g_Ac2(1:3,count)-g_Ac3(1:3,count))./(2*h);
%         dgb2(1:4,count) = (g_Bc2(1:4,count)-g_Bc3(1:4,count))/(2*h);
%     end

% r_A=[60e3;70e3;50e3];
% q_BA= [0.326860225230307;0.522976360368491;0.522976360368491;0.588348405414552];
% 
% [g_A, g_B, Wf, U] = Poly_g_new(r_A, q_BA,Kleopatra)
% 
% dq = Q2DQ(q_BA,r_A,1);
% dqc = dq;
% dqc=norm_dq(dqc);
% 
% r_Ac = 2*cross_quat(dqc(5:8),conj_quat(dqc(1:4)));
% r_Ac = r_Ac(1:3);
% q_BAc = dqc(1:4);
% 
% [g_Ac, g_Bc, Wfc, Uc] = Poly_g_new(r_Ac, q_BAc,Kleopatra)
% 
% diff1 = g_A-g_Ac
% diff2 = g_B-g_Bc
% diff3 = Wf-Wfc
% diff = U-Uc
% end