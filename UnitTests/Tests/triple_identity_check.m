q_BA = [0.8 0.7 0.9 0.1]';
q_BA = q_BA./norm(q_BA);
r_B = [5, 6, 7,0]';
dq = Q2DQ(q_BA,r_B,2);
y_B = [-1,2,-4,0]';
y_B = y_B./norm(y_B);

c = dot(r_B,y_B);
c1 = (cross_quat(q_BA,r_B))'*cross_quat(q_BA,y_B);

M = zeros(8,8);
M(1:4,5:8) = q_tensor(y_B,2)';
M(5:8,1:4) = q_tensor(y_B,2);

c2 = dq'*(M*dq);

r_a = [2,6,7,0]';
r_b = quat_trans(q_BA,r_a,'n');
Mh = zeros(8,8);
Mh(5:8,1:4) = q_tensor(r_a,2);

Ed = zeros(8,8);
Ed(5:8,5:8) =  eye(4);
Eu = zeros(8,8);
Ed(1:4,1:4) =  eye(4);

Er = zeros(8,8);
Er(1:4,5:8) =  eye(4); 
r1 = 2*cross_quat(Ed*dq,Eu*conj_dq(dq,2));
rb2 = Er*cross_dq(dq,Mh*conj_dq(dq,2));
b1 = norm(2*Ed*dq);
z = 2*Ed*dq;
r1 = DQ2R(dq,2);