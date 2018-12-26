function [y] = norm_dq(dq)
%%find the norm of the DQ
dq_conj = conj_dq(dq,2); %second conjugate of dq (q_r* + q_d*e)
dq_mult = cross_dq(dq,dq_conj);

if norm(dq_mult(1:4)) == 1 && dot(dq_mult(1:4),dq_mult(5:8)) <= 1e-5
    y = dq;
else
    q_r = dq(1:4);
    q_d = dq(5:8);
    n = norm(q_r);
    
    q_r_unit = q_r./n;
    q_d_unit = (q_d./n) - dot(q_r./n,q_d./n).*q_r/n;
        
    y = [q_r_unit;q_d_unit];
end
    
end

