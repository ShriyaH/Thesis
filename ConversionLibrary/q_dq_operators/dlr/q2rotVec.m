% q2rotVec.m
% Function to get rotation vector from quaternion. 
% Returns the rotation vector angles.
%
% Inputs:
%     qIn - Array of column vector quaternions in (vector,scalar) form
% 
% Outputs:
%     v_out - Rotation vector (x,y,z) (rad)
%
function dth = q2rotVec(qIn)
%#codegen

sz=size(qIn);
if(sz(1) ~= 4)
    error('Quaternions must be the columns of q');
end

q=q_scalarPos(qIn); % make scalar positive

sa_2 = sqrt(q(1,:).^2 + q(2,:).^2 + q(3,:).^2);
a = asin(sa_2) * 2;

% protect from div-by-0
sa_2 = sa_2 + (sa_2 == 0)*eps;
    
dth = q(1:3,:) .* (ones(3,1) * (a ./ sa_2));
