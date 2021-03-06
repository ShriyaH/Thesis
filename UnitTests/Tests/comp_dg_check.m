function [dga,dgb,g_Bc,dqc] = comp_dg_check(dq)
global Kleopatra 

dga =[];
% dga2 = [];
dgb =[];
% dgb2=[];
count = 1;
    for h = 10.^(-1:16)
        count=count+1;
% [-3.76427582537668e-17 + 0.500000000000000i;-1.74711837371741e-17 + 0.500000000000000i;...
% -2.78887161746535e-17 + 0.500000000000000i;0.00000000000000 + 0.500000000000000i;
% -1.17029587215440e-12 + 0.00000000000000i;7.37502274255400e-13 + 0.00000000000000i;
% -9.90180459364220e-13 + 0.00000000000000i;1.42297405726322e-12 + 0.00000000000000i]
        % complex differentiation
        r_A = 2*cross_quat(dq(5:8),conj_quat(dq(1:4)));
        r_A = r_A(1:3);
        q_BA = dq(1:4);
        [g_A, g_B, Wf, U] = Poly_g_new(r_A, q_BA,Kleopatra);
        
        dqc(1:8,count) = dq + 1i*h;
        dqc(1:8,count) = norm_dq(dqc(1:8,count));

        r_Ac = 2*cross_quat(dqc(5:8,count),conj_quat(dqc(1:4,count)));
        r_Ac = r_Ac(1:3);
        q_BAc = dqc(1:4,count);

        [g_Ac, g_Bc, Wfc, Uc] = Poly_g_new(r_Ac, q_BAc,Kleopatra);
        
        dga(1:3,count) = imag(g_Ac)./h;
        dgb(1:4,count) = imag(g_Bc)./h;
        
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
    end

% r_A=[60e4;70e4;50e4];
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
end