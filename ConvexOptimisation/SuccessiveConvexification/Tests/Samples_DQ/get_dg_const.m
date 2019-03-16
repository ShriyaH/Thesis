function [pd_g] = get_dg_const(dq,g)
q1 = dq(1);
q2 = dq(2);
q3 = dq(3);
q4 = dq(4);
g1 = g(1);
g2 = g(2);
g3 = g(3);


pd_g = zeros(8,8);
pd_g(1:3,1:4) = [2*g1*q1 + 2*g2*q2 + 2*g3*q3, 2*g2*q1 - 2*g1*q2 - 2*g3*q4,2*g3*q1 - 2*g1*q3 + 2*g2*q4, 2*g1*q4 + 2*g2*q3 - 2*g3*q2; 
                2*g1*q2 - 2*g2*q1 + 2*g3*q4, 2*g1*q1 + 2*g2*q2 + 2*g3*q3, 2*g3*q2 - 2*g2*q3 - 2*g1*q4, 2*g3*q1 - 2*g1*q3 + 2*g2*q4;
                2*g1*q3 - 2*g3*q1 - 2*g2*q4,  2*g1*q4 + 2*g2*q3 - 2*g3*q2,  2*g1*q1 + 2*g2*q2 + 2*g3*q3,  2*g1*q2 - 2*g2*q1 + 2*g3*q4];
                           

end