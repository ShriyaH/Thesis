% q_transform.m
% Function to transform vectors using quaternions.
% Uses v_out = q (x) v (x) q^-1
% 
% Inputs:
%     v - array of input vectors, vectors are the columns
%     q - Quaternion to use to transform v, quaternions are the columns
% 
% Outputs:
%     v_out - transformed vector
% 
function [v_out] = q_transform(q,v)
%#codegen
szq=size(q);
szv=size(v);
if(szq(1) ~= 4)
    error('Quaternions must be the columns of q');
end
if(szv(1) ~= 3)
    error('Vectors must be the columns of v');
end
if(~((szv(2) == szq(2)) || (szv(2) == 1) || (szq(2) == 1)))
    error('Either #cols of v==#cols of q, #cols of v==1 or #cols of q==1');
end

temp=q_multiply(q_multiply(q,[v; zeros(1,szv(2))]),q_conjugate(q));
v_out=temp(1:3,:);
