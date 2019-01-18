function [dDWdot] = get_dDWdot(m,dw,dJ,dq,dF,r_F,ns)
global CONSTANTS
g = CONSTANTS.g;
dq_form =CONSTANTS.dq_form;

dDWdot = zeros(8,ns);

%partial derivative with respect to mass 
[U, S, V] = svd(dJ);
dJ_inv = V*(S\U');

pd_dJ = zeros(8,8);
pd_dJ(1:3,5:7) = eye(3);

pd_dJ_inv = zeros(8,8);
pd_dJ_inv(5:7,1:3) = -(1/m^2)*eye(3);

gb = quat_trans(dq(1:4),g,'n');
pd_Fg = [gb;0;0;0;0];

c = omega_tensor(dw,4);

d1 = pd_dJ_inv*(dF - c*(dJ*dw) + [m.*gb;0;0;0;0]) + dJ_inv*( - c*(pd_dJ*dw) + pd_Fg);
 
%partial derivative with respect to DQ
e = omega_tensor(dJ*dw,4);

[pd_g] = get_dg_const(dq,g);
d2 = dJ_inv*(pd_g);

%partial derivative with respect to dual ang. vel. vector
pd_dw = zeros(8,8);
pd_dw(1:3,1:3) = eye(3);
pd_dw(5:7,5:7) = eye(3);
% J1 = dJ(5,1);
% J2 = dJ(6,2);
% J3 = dJ(7,3);
% w1 = dw(1,1);
% w2 = dw(2,1);
% w3 = dw(3,1);
% v1 = dw(5,1);
% v2 = dw(6,1);
% v3 = dw(7,1);
% d3 = [0,(J2*w3 - J3*w3)/J1,(J2*w2 - J3*w2)/J1,0,0,0,0,0;
%       -(J1*w3 - J3*w3)/J2,0,-(J1*w1 - J3*w1)/J2,0,0,0,0,0;
%       (J1*w2 - J2*w2)/J3,(J1*w1 - J2*w1)/J3,0,0,0,0,0,0;
%       0,0,0,0,0,0,0,0;
%       0,-v3,v2,0,0,w3,-w2,0;
%       v3,0,-v1,0,-w3,0,w1,0;
%       -v2,v1,0,0,w2,-w1,0,0;
%       0,0,0,0,0,0,0,0];
d3 = dJ_inv*(-c*(dJ*pd_dw) + e*pd_dw);

%partial derivative with respect to dual thrust vector
pd_dF = zeros(8,8);
pd_dF(1:3,1:3) = eye(3);
pd_dF(5:7,1:3) = omega_tensor(r_F,1);

d4 = dJ_inv*pd_dF;

%build the all columns for ang. vel. pdes
dDWdot(1:8,1) = d1;
dDWdot(1:8,2:9) = d2;
dDWdot(1:8,10:17) = d3;
dDWdot(1:8,18:25) = d4;

end