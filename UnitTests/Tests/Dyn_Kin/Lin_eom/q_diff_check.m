q = [0.5;0.6;0.7;0.4];
q = q./norm(q);
norm(q)
qc = conj_quat(q);

% p = [0.2;0.7;0.3;0.5];
% p = p./norm(p);
% pc = conj_quat(p);

% r = [0.1;0.7;0.6;0.3];
% r = r./norm(r);

s = [-1;-1;-1;1];
s = s./norm(s); 
v = [-1;2;-3;0];

a = cross_quat(cross_quat(s,v),q);
b = cross_quat(cross_quat(qc,v),[0;0;0;1]);
c = a+b;

qq = q_tensor(q,2);
% pp = q_tensor(p,2);
% rr = q_tensor(r,1);
ss = q_tensor(s,1);
qcq = q_tensor(qc,1);

cc = ss*qq*v + qcq*v;
ccc = ((ss*qq)+qcq)*v;

cd = q_tensor(cross_quat(v,q),2)*s;
cdc= cd+qcq*v;


e = qcq*cross_quat(v,q)
e1 = qq*qcq*v

f = div_quat(q,s);
f1 = q_tensor(conj_quat(s),2)*q;
f2 = q_tensor(q,1)*conj_quat(s);
