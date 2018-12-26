function [t] = div_quat(q,r)
%% divide quaternion q by quaternion r
q1= q(1);
q2= q(2);
q3= q(3);
q4= q(4);

r1= r(1);
r2= r(2);
r3= r(3);
r4= r(4);

t1 = (r4*q1 - r1*q4 - r2*q3 + r3*q2)./ norm(r)^2;
t2 = (r4*q2 + r1*q3 - r2*q4 - r3*q1)./ norm(r)^2;
t3 = (r4*q3 - r1*q2 + r2*q1 - r3*q4)./ norm(r)^2;
t4 = (r4*q4 + r1*q1 + r2*q2 + r3*q3)./ norm(r)^2;

t = [t1;t2;t3;t4];
end