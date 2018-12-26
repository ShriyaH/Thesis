function q_conj = conj_quat(q)
%Unit quaternion conjugation

q_vect= q(1:3);
q4= q(4);

q_conj = [-q_vect; q4];
end