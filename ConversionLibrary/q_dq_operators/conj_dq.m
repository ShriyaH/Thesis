function dq_conj = conj_dq(dq,i)
%%'i' gives the type of dq conjugate: 1,2,3
p_vect = dq(1:3);
p4 = dq(4);
q_vect = dq(5:7);
q4 = dq(8);

switch i
    case 1
        dq_conj = [p_vect; p4; -q_vect; -q4];  %first conjugate of dq (q_r - q_d*e)
    case 2
        dq_conj = [-p_vect; p4; -q_vect; q4];  %second conjugate of dq (q_r* + q_d*e)
    case 3
        dq_conj = [-p_vect; p4; q_vect; -q4];  %third conjugate of dq (q_r* - q_d*e) (used for dq transformation)
    otherwise
        disp('Wrong selection of type of conjugate');
end
 
end