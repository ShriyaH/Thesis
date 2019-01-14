q = [0.8;0.9;0.7;0.5];
q = q./norm(q);
a = q_tensor(q,1);
b = q_tensor(q,2);
c = q_tensor(conj_quat(q),1);
d = q_tensor(conj_quat(q),2);
e=[2;3;4];
f = quat_trans(q,e,'vect');
g = quat_trans(conj_quat(q),f,'vect');
e=[2;3;4;0];
h = cross_quat(q,e);
i = q_tensor(e,2)*q;

w = [3;4;5;0;6;7;8;0];
dq = Q2DQ(q,e,2);
W = omega_tensor(w,3);
qdot = 0.5.*W*dq

qq = q_tensor(q,2);
Q = zeros(8,8);
Q(1:4,1:4) =qq;
Q(5:8,5:8) =qq;
qqd = q_tensor(dq(5:8),2);
Q(5:8,1:4) = qqd;
% Q(4,:) = 0;
% Q(8,:) = 0;
qdot1 = 0.5.*Q*w