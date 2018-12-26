% q_multiply.m
% Function to multiply two quaternions together using the (x) opperator.  
% The result is the same as multiplying the DCM versions of the quaternions 
% in the same order.  I.E.
% Q2DCM(q_out) = Q2DCM(q1) * Q2DCM(q2) = Q2DCM(q1 (x) q2) = Q2DCM(q_multiply(q1,q2)
%
% Inputs:
%     q1 - Left side quaternions in (vector,scalar) form, array of
%     quaternion columns
%     q2 - Right side quaternion in (vector,scalar) form, array of
%     quaternion columns
%
% Outputs:
%     q_out - Product quaternion in (vector,scalar) form
% 
% Notes:
% See J. R. Wertz, "Spacecraft Attitude Determination and Control", 1978
% eq D-13
function [q_out]=q_multiply(q1,q2)
%#codegen

sz1=size(q1);
sz2=size(q2);
if((sz1(1) ~= 4)||(sz2(1) ~= 4))
    error('Quaternions must be the columns of the inputs');
end
if(~((sz1(2) == sz2(2)) || (sz1(2) == 1) || (sz2(2) == 1)))
    error('Either #cols of q1==#cols of q2, #cols of q1==1 or #cols of q2==1');
end

q_out=zeros(4,max([sz1(2) sz2(2)])); %set the size of q_out

q_out(1,:) = q1(4,:).*q2(1,:) + q2(4,:).*q1(1,:) - q1(2,:).*q2(3,:) + q1(3,:).*q2(2,:);
q_out(2,:) = q1(4,:).*q2(2,:) + q2(4,:).*q1(2,:) - q1(3,:).*q2(1,:) + q1(1,:).*q2(3,:);
q_out(3,:) = q1(4,:).*q2(3,:) + q2(4,:).*q1(3,:) - q1(1,:).*q2(2,:) + q1(2,:).*q2(1,:);
q_out(4,:) = q1(4,:).*q2(4,:) - q1(1,:).*q2(1,:) - q1(2,:).*q2(2,:) - q1(3,:).*q2(3,:);
