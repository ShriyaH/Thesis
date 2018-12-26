 function [ DCM] = Q2DCM( Q )
%% Function to create a DCM from a quaternion

q1 = Q(1);
q2 = Q(2);
q3 = Q(3);
q4 = Q(4);

E =[0 -q3 q2;
   q3 0 -q1;
  -q2 q1 0;];

DCM = (q4^2-norm([q1 q2 q3])^2)*eye(3)+2*[q1 q2 q3]'*[q1 q2 q3]-2*q4*E;

end

