function [v] = quat_trans(p,q,out)
%% transform vector with quaternion, p = 4x1 (quaternion), q = vect , out = 'vect' gives in vector format
    if norm(p)~=1
        p = p./norm(p);
    end
    if size(p)== [1 4]
        p = p';
    end
    if size(q)== [1 4]
        q = q';
    elseif size(q) == [1 3]
        q = q';
        q = [q;0];
    elseif size(q) == [3 1]
        q = [q;0];
    end

    pc = conj_quat(p);

    v = cross_quat(p,cross_quat(q,pc));
    % v = v(1:3,1)';

    if out == 'vect'
        v = v(1:3,1)';
    else
        v = v;
    end
    
end