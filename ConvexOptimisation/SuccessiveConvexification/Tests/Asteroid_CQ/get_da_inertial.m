function [da] = get_da_inertial(T,q,m,ns)
q = q./norm(q);
da = zeros(3,ns);
%wrt m
d1 =[-((q(1)^2-q(2)^2-q(3)^2+q(4)^2)*T(1)+(2*T(2)*q(2)+2*T(3)*q(3))*q(1)-2*q(4)*(T(2)*q(3)-T(3)*q(2)));
       -((-q(1)^2+q(2)^2-q(3)^2+q(4)^2)*T(2)+(2*T(1)*q(2)-2*T(3)*q(4))*q(1)+2*q(3)*(T(1)*q(4)+T(3)*q(2)));
       -((-q(1)^2-q(2)^2+q(3)^2+q(4)^2)*T(3)+(2*T(1)*q(3)+2*T(2)*q(4))*q(1)-2*q(2)*(T(1)*q(4)-T(2)*q(3)))];
da1 = d1./(m^2);

%wrt q
d2 = [T(1)*q(1)+T(2)*q(2)+T(3)*q(3), -T(1)*q(2)+T(2)*q(1)+T(3)*q(4), -T(1)*q(3)-T(2)*q(4)+T(3)*q(1), T(1)*q(4)-T(2)*q(3)+T(3)*q(2); 
      T(1)*q(2)-T(2)*q(1)-T(3)*q(4), T(1)*q(1)+T(2)*q(2)+T(3)*q(3), T(1)*q(4)-T(2)*q(3)+T(3)*q(2), T(1)*q(3)+T(2)*q(4)-T(3)*q(1); 
      T(1)*q(3)+T(2)*q(4)-T(3)*q(1), -T(1)*q(4)+T(2)*q(3)-T(3)*q(2), T(1)*q(1)+T(2)*q(2)+T(3)*q(3), -T(1)*q(2)+T(2)*q(1)+T(3)*q(4)];

da2 = 2.*d2./m;

%wrt T
d3 = [(q(1)^2-q(2)^2-q(3)^2+q(4)^2),(2*q(1)*q(2)-2*q(3)*q(4)), (2*q(1)*q(3)+2*q(2)*q(4));
    (2*q(1)*q(2)+2*q(3)*q(4)), (-q(1)^2+q(2)^2-q(3)^2+q(4)^2), (-2*q(1)*q(4)+2*q(2)*q(3));
    (2*q(1)*q(3)-2*q(2)*q(4)), (2*q(1)*q(4)+2*q(2)*q(3)), (-q(1)^2-q(2)^2+q(3)^2+q(4)^2)];
da3 = d3./m;

da(1:3,1) = da1;
da(1:3,8:11) = da2;
da(1:3,15:17) = da3;
end