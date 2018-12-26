function y = cross_quat(q,p)
% Quaternion Cross Product

q4= q(4);
p4= p(4);

q_vect= q(1:3);
p_vect= p(1:3);

y = [q4.*p_vect + p4.*q_vect - cross(q_vect, p_vect); q4*p4 - dot(q_vect,p_vect)];
end


