function [p_trans] = transform_dq(dq,r) 
%%dq,r are the DQ for transforming and the vector or quaternion or DQ format

if size(r) == [1,3]
        dp = [0;0;0;1;r';0]; %Pure DQ form of vector to be transformed
elseif size(r) == [3,1]
        dp = [0;0;0;1;r;0]; 
elseif size(r) == [1,8]
        dp = r';
elseif size(r) == [8,1]
        dp = r;
end

dq_unit = dq_norm(dq);

dq_c = conj_dq(dq_unit,3); %3rd conjugate of the DQ

int = cross_dq(dq_unit,dp);
p_trans = cross_dq(int,dq_c);
  
end


