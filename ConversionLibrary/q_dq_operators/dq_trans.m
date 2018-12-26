function [v] = dq_trans(p,q)
%% transform dual vector with dual quaternion, p = 4x1 (dq), q = dual vect 
    if size(p)== [1 8]
        p = p';
    end
    if size(q)== [1 8]
        q = q';
    end

    p = norm_dq(p);    
    pc = conj_dq(p,3);

    v = cross_dq(p,cross_dq(q,pc));
    
end