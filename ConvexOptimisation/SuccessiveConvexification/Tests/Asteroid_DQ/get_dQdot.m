function [dDQdot] = get_dQdot(dw,dq,ns)

%initialise
dDQdot = zeros(8,ns);

%partial derivative with respect to ang. vel.
d1 = zeros(8,8);

d1a = q_tensor(dq(1:4),2); %Quat dot product
d1b = q_tensor(dq(5:8),2); %Quat dot product

d1(1:4,1:4) = d1a;
d1(5:8,5:8) = d1a;
d1(5:8,1:4) = d1b;

dDQdot1 = d1./2;


%partial derivative with respect to DQ
pdq = omega_tensor(dw,3); %Omega (ang. vel DQ cross product)

pw = cross_quat(dq(1:4),dw(5:8));
pdw = [0 0 0 0 0 0 0 0 
         0 0 0 0 0 0 0 0 
         0 0 0 0 0 0 0 0
         0 0 0 0 0 0 0 0
         -pw(4) -pw(3) pw(2) pw(1) 0 0 0 0 
         pw(3) -pw(4) -pw(1) pw(2) 0 0 0 0 
         -pw(2) pw(1) -pw(4) pw(3) 0 0 0 0
          0 0 0 0 0 0 0 0];
pdw = d1.*pdw;

d2 = (pdq+pdw)./2;

dDQdot2 = d2;

dDQdot(1:8,2:9) = dDQdot2;
dDQdot(1:8,10:17) = dDQdot1;

end