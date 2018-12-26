function [q_BI] = Att_kin(w_BI)
q_BI0 = [0 0 0 1]';
W = [0 w_BI(3) -w_BI(2) w_BI(1)
     -w_BI(3) 0 w_BI(1) w_BI(2)
     w_BI(2) -w_BI(1) 0 w_BI(3)
     -w_BI(1) -w_BI(2) -w_BI(3) 0];


T = [0 2000];
Opt = odeset('AbsTol',1e-10,'RelTol',2.3e-10);
[t,q_BI] = ode45(@att_kin, T, q_BI0,Opt);  

    function q_BI_dot = att_kin(T,q_BI)
        q_BI_dot = 0.5*W*q_BI;
    end

end