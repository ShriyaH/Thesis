function [dqdot] = get_dqdot(w,q,ns)

dqdot = zeros(4,ns);

d1 =[0,w(3),-w(2),w(1);
    -w(3),0,w(1),w(2);
    w(2),-w(1),0,w(3);
    -w(1),-w(2),-w(3),0];
dqdot1 = d1;

d2 = [q(4), -q(3), q(2);
      q(3), q(4), -q(1);
     -q(2), q(1), q(4);
     -q(1), -q(2), -q(3)]; 
  
dqdot2 = d2;


dqdot(1:4,8:11) = dqdot1;
dqdot(1:4,12:14) = dqdot2;

end