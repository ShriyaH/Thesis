function [r] = DQ2R(dq,dq_form)
    if dq_form ==1
        r = 2.*cross_quat(dq(5:8),conj_quat(dq(1:4)));
    else
        r = 2.*cross_quat(conj_quat(dq(1:4)),dq(5:8));
    end
end