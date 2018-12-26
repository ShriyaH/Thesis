function [dJ] = dq_inertia(m,I)
%gives the 8x8 inertia matrix for dq dynamics

m1 = [m*eye(3) zeros(3,1)
     zeros(1,3) 1];
 
J1 = [I zeros(3,1)
     zeros(1,3) 1];
 
x = zeros(4,4); 

dJ = [x, m1;
      J1, x];
end