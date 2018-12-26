% Function to change the scalar part of a quaternion to be positive
%q can be an array of quaternions
%quaternions must be column vectors
function [q_out] = q_scalarPos(q)

sz=size(q);
if(sz(1) ~= 4)
    error('Quaternions must be the columns of q');
end

q_out=q;
temp = q(4,:) < 0;
q_out(:,temp)=-q_out(:,temp);


