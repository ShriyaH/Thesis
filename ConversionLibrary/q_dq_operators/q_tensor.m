function y = q_tensor(q,n)
% n=1 [q x], n=2 [q .]
q1= q(1);
q2= q(2);
q3 = q(3);
q4 = q(4);

switch n
    case 1
        y = [q4 q3 -q2 q1
             -q3 q4 q1 q2
             q2 -q1 q4 q3
             -q1 -q2 -q3 q4];
    case 2
        y = [q4 -q3 q2 q1
             q3 q4 -q1 q2
             -q2 q1 q4 q3
             -q1 -q2 -q3 q4];

otherwise
        disp('Wrong selection of type of tensor');
end
 
end
