function [dDQdot] = get_dQdot(dw,dq,ns)

%initialise
dDQdot = zeros(8,ns);

%partial derivative with respect to DQ
d1 =omega_tensor(dw,3); %Omega (ang. vel DQ cross product)
dDQdot1 = d1./2;

%partial derivative with respect to ang. vel.
d2 = zeros(8,8);

d2a = q_tensor(dq(1:4),2); %Quat dot product
d2b = q_tensor(dq(5:8),2); %Quat dot product

d2(1:4,1:4) = d2a;
d2(5:8,5:8) = d2a;
d2(5:8,1:4) = d2b;

dDQdot2 = d2./2;

dDQdot(1:8,2:9) = dDQdot1;
dDQdot(1:8,10:17) = dDQdot2;

end