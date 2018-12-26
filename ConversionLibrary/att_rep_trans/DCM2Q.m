% DCM2Q.m
% Function to convert DCM to quaternion.  Uses the most numerically accurate 
% method depending on DCM values.
%
% Inputs:
%     DCM - The DCM representation of q
% 
% Outputs:
%     q - The quaternion to use in (vector,scalar) form
% 
% Notes:
% See J. R. Wertz, "Spacecraft Attitude Determination and Control", 1978
% eq 12-14
function q = DCM2Q(DCM)
%#eml
q=zeros(4,1);
b=zeros(4,1);

tr=DCM(1,1)+DCM(2,2)+DCM(3,3);
b(1)=(1+tr)*0.25;
b(2)=(1+2*DCM(1,1)-tr)*0.25;
b(3)=(1+2*DCM(2,2)-tr)*0.25;
b(4)=(1+2*DCM(3,3)-tr)*0.25;

[v,i]=max(b);
switch i,
case 1
   q(4)=sqrt(b(1));
   temp=0.25/q(4);
   q(1)=(DCM(2,3)-DCM(3,2))*temp;
   q(2)=(DCM(3,1)-DCM(1,3))*temp;
   q(3)=(DCM(1,2)-DCM(2,1))*temp;
case 2
   q(1)=sqrt(b(2));
   temp=0.25/q(1);
   q(4)=(DCM(2,3)-DCM(3,2))*temp;
   q(2)=(DCM(2,1)+DCM(1,2))*temp;
   q(3)=(DCM(3,1)+DCM(1,3))*temp;
case 3
   q(2)=sqrt(b(3));
   temp=0.25/q(2);
   q(4)=(DCM(3,1)-DCM(1,3))*temp;
   q(1)=(DCM(1,2)+DCM(2,1))*temp;
   q(3)=(DCM(2,3)+DCM(3,2))*temp;
case 4
   q(3)=sqrt(b(4));
   temp=0.25/q(3);
   q(4)=(DCM(1,2)-DCM(2,1))*temp;
   q(1)=(DCM(3,1)+DCM(1,3))*temp;
   q(2)=(DCM(2,3)+DCM(3,2))*temp;
end

if(q(4)<0)
  q=-q;
end

