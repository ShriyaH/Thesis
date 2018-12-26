function [y] = cross_dnum(p,q)

p_r = p(1:4);
p_d = p(5:8);

q_r = q(1:4);
q_d = q(5:8);

y = [cross_quat(p_r,q_r); (cross_quat(p_r,q_d) + cross_quat(p_d,q_r))];
end


