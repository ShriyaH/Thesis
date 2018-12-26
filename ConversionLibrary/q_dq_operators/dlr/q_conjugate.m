%quaternion conjugate
%q can be an array of quaternions
%quaternions must be column vectors
function [q_out]=q_conjugate(q)
% %#codegen

sz=size(q);
if(sz(1) ~= 4)
    error('Quaternions must be the columns of q');
end

q_out=q;
q_out(1:3,:)=-q(1:3,:);
end

