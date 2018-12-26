dq = sym('q', [1 8]).';
dw1 = sym('w', [1 3]).';
dw2 = sym('v', [1 3]).';
dw = [dw1;0;dw2;0];
dW1 = sym('W', [1 3]).';
dW2 = sym('V', [1 3]).';
dW = [dW1;0;dW2;0];


syms cross(x,y) 
x = sym('x', [1 4]).';
y = sym('y', [1 4]).';
cross(x,y) = [x(2).*y(3)-x(3).*y(1,2);
              x(3).*y(1)-x(1).*y(3);
              x(1).*y(2)-x(2).*y(1)];

% syms dqc(dq) 
% dqc(dq) = [-dq(1:3), dq(4), dq(5:7), -dq(8)];
% syms cross_dq(dq,v)
% cross_dq(dq,v) = [cross_quat(dq(1:4),v(1:4)), (cross_quat(dq(1:4),v(5:8)) + cross_quat(dq(5:8),v(1:4)))];
