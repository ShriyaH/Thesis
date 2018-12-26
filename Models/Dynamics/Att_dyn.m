function [w_BI] = Att_dyn(T_D,T_C,I)
w_BI0 = [0 0 0]';
I_inv = inv(I);

T = [0 2000];

Opt = odeset('AbsTol',1e-10,'RelTol',2.3e-10);
[t,w_BI] = ode45(@att_dyn, T, w_BI0,Opt);  

    function w_BI_dot = att_dyn(T,w_BI)
        w_BI_dot = I_inv * (cross(w_BI, I*w_BI) + T_D + T_C);
    end


end