% q = [0.8;0.9;0.7;0.5];
% q = q./norm(q);
% a = q_tensor(q,1);
% b = q_tensor(q,2);
% c = q_tensor(conj_quat(q),1);
% d = q_tensor(conj_quat(q),2);
% e=[2;3;4];
% f = quat_trans(q,e,'vect');
% g = quat_trans(conj_quat(q),f,'vect');
% e=[2;3;4;0];
% h = cross_quat(q,e);
% i = q_tensor(e,2)*q;
% 
% w = [3;4;5;0;6;7;8;0];
% dq = Q2DQ(q,e,2);
% W = omega_tensor(w,3);
% qdot = 0.5.*W*dq
% 
% qq = q_tensor(q,2);
% Q = zeros(8,8);
% Q(1:4,1:4) =qq;
% Q(5:8,5:8) =qq;
% qqd = q_tensor(dq(5:8),2);
% Q(5:8,1:4) = qqd;
% % Q(4,:) = 0;
% % Q(8,:) = 0;
% qdot1 = 0.5.*Q*w

% syms q1 q2 q3 q4 g1 g2 g3
% A = [q4, q3, -q2, q1;
%     -q3, q4, q1, q2;
%      q2, -q1, q4, q3
%     -q1,-q2, -q3, q4];
% B = [0, g3, -g2, g1;
%     -g3, 0, g1, g2;
%     g2, -g1,0,g3;
%     -g1,-g2,-g3,0];
% C = [-q1;-q2;-q3;q4];
% 
% D = A*(B*C);
% 
% d1 = diff(D,q1)
% d2 = diff(D,q2)
% d3 = diff(D,q3)
% d4 = diff(D,q4)

syms m J1 J2 J3 w1 w2 w3 v1 v2 v3 F1 F2 F3 r1 r2 r3 T1 T2 T3

A = [0,0,0,0,m,0,0,0;
     0,0,0,0,0,m,0,0;
     0,0,0,0,0,0,m,0;
     0,0,0,0,0,0,0,1;
     J1,0,0,0,0,0,0,0;
     0,J2,0,0,0,0,0,0;
     0,0,J3,0,0,0,0,0;
     0,0,0,1,0,0,0,0];
B = inv(A)

d = diff(B,m)

C = [w1;w2;w3;0;v1;v2;v3;0];
D = [0 -w3 w2 w1 0 0 0 0 
     w3 0 -w1 w2 0 0 0 0 
     -w2 w1 0 w3 0 0 0 0
     0 0 0 0 0 0 0 0
     0 -v3 v2 v1 0 -w3 w2 w1
     v3 0 -v1 v2 w3 0 -w1 w2
     -v2 v1 0 v3 -w2 w1 0 w3
     0 0 0 0 0 0 0 0];
 
 W= B*(-D*(A*C))
 
d1 = diff(W,w1)
d2 = diff(W,w2)
d3 = diff(W,w3) 
d4 = diff(W,v1)
d5 = diff(W,v2)
d6 = diff(W,v3)

E = [0,-r3,r2;
    r3,0,-r1;
    -r2,r1,0];
T1 = E(1,:)*[F1;F2;F3];
T2 = E(2,:)*[F1;F2;F3];
T3 = E(3,:)*[F1;F2;F3];

G = B*[F1;F2;F3;0;T1;T2;T3;0];

d7 = diff(G,F1)
d8 = diff(G,F2)
d9 = diff(G,F3)
