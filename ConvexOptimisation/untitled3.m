dq = [1,2,3,4,5,6,7,8]';
d2 = zeros(8,8);

d2a = q_tensor(dq(1:4),2); %Quat dot product
d2b = q_tensor(dq(5:8),2); %Quat dot product

d2(1:4,1:4) = d2a;
d2(5:8,5:8) = d2a;
d2(5:8,1:4) = d2b;
d2(:,4) = zeros(8,1);
d2(:,8) = zeros(8,1);
dDQdot2 = d2./2;