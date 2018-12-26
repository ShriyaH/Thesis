function [ q3 ] = Q_mult( q1,q2 )

q1_v=q1(1:3);
q1_s=q1(4);

q2_v=q2(1:3);
q2_s=q2(4);

q3=[q1_s*q2_v+q2_s*q1_v-cross(q1_v,q2_v);
    q1_s*q2_s-q1_v'*q2_v;];



end

