%%%%%%%%%%%%%%%%%%%%%     ScLERP     %%%%%%%%%%%%%%%%%%%%%%%
% This routine aims for calculating a unit dual quaternion,  describing a rotation matrix,
% which lies between two known unit dual quaternions - dq1 and dq2, using a screw linear interpolation - Sclerp.
% Sclerp follow the shortest great arc on a unit 8D hyper-sphere, hence, the shortest possible interpolation path.
% Consequently, Sclerp has constant angular velocity, so it is the optimal interpolation curve between two rotations and translations.
% sclerp(dq1, dq2, t) = dq1*(cos(tau*dtheta/2) + dl * sin(tau*dtheta/2))
% where theta is the angle between the two unit dual quaternions,
% and tau is between [0,1]
% Shriya Hazra %

function [dqm,r,r_norm,ang] = ScLERP(dqi, dqn, tt)
%       where qi=[x1 y1 z1 w1 a1 b1 c1 d1] - start unit quaternions
%             qn=[x2 y2 z2 w2 a2 b2 c2 d2] - end unit quaternions
%             t=[0 to 1]

T = tt./tt(end);
dqi = norm_dq(dqi);
dqn = norm_dq(dqn);

dqi_c = conj_dq(dqi,2);       % conjugate of dual quaternion for ScLERP
dqs = cross_dq(dqi_c,dqn);    % ScLERP operation
dqs = norm_dq(dqs);           %check norm of dual quaternion and if not normalise

t = 2.*cross_quat(dqs(5:8),conj_quat(dqs(1:4)));  % translation vector from dual quaternion
t = t(1:3);
theta = 2*acos(dqs(4));             % degree of rotation from dual quaternion
l = dqs(1:3)./sin(theta/2);             % axis of screw motion
m = 1/2*(cross(t,l) + cross(l,cross(t,l)).*cot(theta/2)); % moment vector of screw motion
d = l'*t;                           % pitch of screw
        
for i = 1:length(T)
    if T(i)==0  % initial point=> dqm=dqi
        dqm(1:8,i) = dqi;
%     elseif T(i)==1 % final time => dqm=dqn
%         dqm(1:8,i) = dqn;
    else         
        dq1 = [cos(T(i)*theta/2); (T(i)*d/2)*(-sin(T(i)*theta/2))];
        dq2 = [sin(T(i)*theta/2).*l; ((T(i)*d/2)*cos(T(i)*theta/2)).*l + sin(T(i)*theta/2).*m];
        dq = [dq2(1:3);dq1(1);dq2(4:6);dq1(2)];
        
        dqm(1:8,i) = cross_dq(dqi,dq);
        dqm(1:8,i) = norm_dq(dqm(1:8,i));
    end
    r(1:4,i) = 2.*cross_quat(dqm(5:8,i),conj_quat(dqm(1:4,i)));
    ang(i) = 2*rad2deg(acos(dqm(4,i)));
    r_norm(i) = norm(r(1:3,i));
end
end




