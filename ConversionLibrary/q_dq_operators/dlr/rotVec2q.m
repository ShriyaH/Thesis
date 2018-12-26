% rotVec2q.m
% Function to get quaternion from rotation vector. 
% Returns the rotation vector angles.
%
% Inputs:
%     dth - Array of rotation vectors (x,y,z) (rad)
% 
% Outputs:
%     q - Array of column vector quaternions in (vector,scalar) form
%
function q = rotVec2q(dth)
%#eml

sz=size(dth);
if(sz(1) ~= 3)
    error('Input must be columns of rotation vectors.');
end

sz=size(dth);

alpha=sqrt(sum(dth.*dth));
if(alpha==0)
    idx=find(alpha==0);
    alpha(idx)=eps;
end
u=dth./(ones(sz(1),1)*alpha);

q=[u.*(ones(sz(1),1)*sin(alpha/2)); ...
   cos(alpha/2)];

