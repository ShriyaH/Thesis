function [dgac,dgbc,dgaf,dgbf] = get_dG(dq,h)
global Kleopatra CONSTANTS
dq_norm = CONSTANTS.dq_norm;
dgac =[];
dgaf = [];
dgbc =[];
dgbf=[];
count = 0;
%     for h = 10.^(-1:16)
%         count=count+1;

        % complex differentiation
        dqc(1:8,count) = dq + 1i*h;
        dqc=norm_dq(dqc(1:8,count));

        r_Ac = DQ2R(dqc,dq_norm);
        r_Ac = r_Ac(1:3);
        q_BAc = dqc(1:4);

        [g_Ac(1:3,count), g_Bc(1:4,count), Wfc(count), Uc(count)] = Poly_g_new(r_Ac, q_BAc,Kleopatra);

        dgac(1:3,count) = imag(g_Ac(1:3,count))./h;
        dgbc(1:4,count) = imag(g_Bc(1:4,count))./h;

        % finite differentiation
        dqc2(1:8,count) = dq + h;
        dqc2=norm_dq(dqc2(1:8,count));

        dqc3(1:8,count) = dq - h;
        dqc3=norm_dq(dqc3(1:8,count));

        r_Ac2 = DQ2R(dqc2,dq_norm);
        r_Ac2 = r_Ac2(1:3);
        q_BAc2 = dqc2(1:4);

        [g_Ac2(1:3,count), g_Bc2(1:4,count), Wfc2(count), Uc2(count)] = Poly_g_new(r_Ac2, q_BAc2,Kleopatra);

        r_Ac3 = DQ2R(dqc3,dq_norm);
        r_Ac3 = r_Ac3(1:3);
        q_BAc3 = dqc3(1:4);

        [g_Ac3(1:3,count), g_Bc3(1:4,count), Wfc3(count), Uc3(count)] = Poly_g_new(r_Ac3, q_BAc3,Kleopatra);

        dgaf(1:3,count) = (g_Ac2(1:3,count)-g_Ac3(1:3,count))./(2*h);
        dgbf(1:4,count) = (g_Bc2(1:4,count)-g_Bc3(1:4,count))/(2*h);
%     end

end