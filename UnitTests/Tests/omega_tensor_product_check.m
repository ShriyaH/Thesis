J = zeros(8,8);
J(1:3,5:7) = 3*eye(3);
J(5,1) = 10;
J(6,2) = 20;
J(7,3) = 15;
J(4,8) = 1;
J(8,4) = 1;

wb = [1;3;2;0;4;5;6;0];
wa = [2;5;3;0;0;0;0;0];
v = J*wb;
d = omega_tensor(wa,4)*(v);
e = -omega_tensor(v,4)*(wa);
f = d-e

g = J*(omega_tensor(wa,4)*wb);
h = -J*(omega_tensor(wb,4)*wa);
i = g-h

r = [0;0;0;0;3;4;5;6];
j = J*(omega_tensor(wa,4)*(omega_tensor(wa,4)*r));
k = -J*(omega_tensor(wa,4)*omega_tensor(r,4)*wa);
l = -J *omega_tensor(omega_tensor(wa,4)*r,4)*wa;
m = j-k
n = k-l
o = j-l