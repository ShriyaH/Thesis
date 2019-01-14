%%%%%%%%%%%%%%%%%%%%%     ScLERP     %%%%%%%%%%%%%%%%%%%%%%%
% This routine aims for calculating a unit dual quaternion,  describing a rotation matrix,
% which lies between two known unit dual quaternions - dq1 and dq2, using a screw linear interpolation - Sclerp.
% Sclerp follow the shortest great arc on a unit 8D hyper-sphere, hence, the shortest possible interpolation path.
% Consequently, Sclerp has constant angular velocity, so it is the optimal interpolation curve between two rotations and translations.
% sclerp(dq1, dq2, t) = dq1*(cos(tau*dtheta/2) + dl * sin(tau*dtheta/2))
% where theta is the angle between the two unit dual quaternions,
% and tau is between [0,1]
% Shriya Hazra %
function [dqm,r,r_norm,ang,l,m,d,theta,t] = ScLERP(dqi, dqn, tt)
%       where qi=[x1 y1 z1 w1 a1 b1 c1 d1] - start unit quaternions
%             qn=[x2 y2 z2 w2 a2 b2 c2 d2] - end unit quaternions
%             t=[0 to 1]
global CONSTANTS
dq_form = CONSTANTS.dq_form;

T = tt./tt(end);
dqi = norm_dq(dqi);
dqn = norm_dq(dqn);

dqi_c = conj_dq(dqi,2);       % conjugate of dual quaternion for ScLERP
if dqi(1:4)~= dqn(1:4)
    dqs = cross_dq(dqi_c,dqn);    % ScLERP operation
    dqs = norm_dq(dqs);           %check norm of dual quaternion and if not normalise 


    t = DQ2R(dqs,dq_form);  % translation vector from dual quaternion
    t = t(1:3);
    % t = [8;-8;-1];
    theta = 2*acos(dqs(4));             % degree of rotation from dual quaternion
    l = dqs(1:3)./sin(theta/2);         % axis of screw motion
    d = dot(l',t',2);           % pitch of screw
    m = (1/2).*(cross(t,l) + (t-d.*l).*cot(theta/2)); % moment vector of screw motion

    for i = 1:length(T)
        if T(i)==0  % initial point=> dqm=dqi
            dqm(1:8,i) = dqi;        
        elseif T(i)==1 % final time => dqm=dqn
            dqm(1:8,i) = dqn;        
        else         
            dq1 = [l.*sin(T(i)*theta/2); cos(T(i)*theta/2)];
            dq2 = [(m.*sin(T(i)*theta/2) + (l.*(T(i)*d/2*cos(T(i)*theta/2)))) ; -(T(i)*d/2)*(sin(T(i)*theta/2))];
            dq = [dq1;dq2];
            dq = norm_dq(dq);

            dqm(1:8,i) = cross_dq(dqi,dq);
            dqm(1:8,i) = norm_dq(dqm(1:8,i));       
        end
        r(1:4,i) = DQ2R(dqm(1:8,i),dq_form);
        ang(i) = 2*rad2deg(acos(dqm(4,i)));
        r_norm(i) = norm(r(1:3,i));
    end
else
    ri =  DQ2R(dqi,dq_form);
    rf =  DQ2R(dqn,dq_form);
    for i = 1:length(T)
        K=length(T);
        rm = ((K-i-1)/(K-1))*ri + (i/(K-1))*rf;
        dqm(1:8,i) = Q2DQ(dqi(1:4),rm,dq_form);
    end
end
ri =  DQ2R(dqi,dq_form);
rf =  DQ2R(dqn,dq_form);
    for i = 1:length(T)
        K=length(T);
        rm(1:4,i) = ((K-i-1)/(K-1))*ri + (i/(K-1))*rf;
        dqmm(1:8,i) = Q2DQ(dqm(1:4,i),rm(1:4,i),dq_form);
        ang(i) = 2*rad2deg(acos(dqmm(4,i)));
        r_n(i) = norm(rm(1:4,i));
    end

% figure()
% plot(1:40,ang','Marker','.')
% xlabel('Time (sec)')
% ylabel('Angle (deg)')
% grid on  
% 
% figure()
% plot(1:40,r_n,'Marker','.')
% xlabel('Time (sec)')
% ylabel('Angle (deg)')
% grid on 

% figure()
% plot3(rm(1,:),rm(2,:),rm(3,:),'Marker','.')
% xlabel('X (m)')
% ylabel('Y (m)')
% zlabel('Z (m)')
% grid on 
end


