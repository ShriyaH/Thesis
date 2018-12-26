function dq_conj2 = conj_dquat2(q)
%Dual quaternion conjugation (q_r^* - q_d^*e)

q1= q(1:4);
q10= q1(4);
q1vect= q1(1:3);

q2= q(5:8);
q20= q2(4);
q2vect= q2(1:3);

dq_conj2 = [-q1_vect; q10; q2_vect; -q20];