function [dDWdot] = get_dWdot(m,dw,dJ,dq,dF,wa,ns,Asteroid)
%partial differential of w_bi_b wrt to state

global Switch CONSTANTS

dq_form =CONSTANTS.dq_form;
r_F =CONSTANTS.r_F;
dDWdot = zeros(8,ns);

if Switch.constant_grav 
    g = CONSTANTS.g;
    gb = quat_trans(dq(1:4),g,'n');
    dFg = [m.*gb;0;0;0;0];
else
    %polyhedron gravity field
    [gb, Tgb] = Get_pertforces(q_k,Asteroid);
    dFg = [m.*gb;0;Tgb;0];
end


dW(1:4) = quat_trans(dq(1:4),wa(1:4),'n');
dW(5:8) = quat_trans(dq(1:4),wa(5:8),'n');
dW = dW';
R = DQ2R(dq,dq_form);
R = [0;0;0;0;R];

%% partial derivative with respect to mass 
[U, S, V] = svd(dJ);
dJ_inv = V*(S\U');

pd_dJ = zeros(8,8);
pd_dJ(1:3,5:7) = eye(3);

pd_dJ_inv = zeros(8,8);
pd_dJ_inv(5:7,1:3) = -(1/m^2)*eye(3);

if Switch.constant_grav
    pd_Fg = [gb;0;0;0;0];
else
    pd_Fg = [gb;0;pd_Tgb;0];
end

a = omega_tensor((dw+dW),4);
b = omega_tensor(dW,4);
c = omega_tensor(dw,4);

d1 = pd_dJ_inv*(dF + dFg -a*(dJ*(dw+dW))- dJ*(b*dw) - dJ*(b*(b*R))) + ...
     dJ_inv*(pd_Fg - a*(pd_dJ*(dw+dW)) - pd_dJ*(b*dw) - pd_dJ*(b*(b*R)));

%% partial derivative with respect to DQ
d = omega_tensor(R,4);
e = omega_tensor(dJ*dw,4);
f = omega_tensor(dJ*dW,4);

[pd_dW,pd_R] = quat_rot_pde(dq,wa);
if Switch.constant_grav
    [pd_g] = get_dg_const(dq,g); 
else
    [dgac,dgbc,dgaf,dgbf] = get_dG(dq,h);
    pd_g = [dgbc;0;0;0;0;];
end

d2 = dJ_inv*(pd_g - c*(dJ*pd_dW) + e*pd_dW + f*pd_dW - b*(dJ*pd_dW) + dJ*(c*pd_dW) ...
             - dJ*(b*(b*pd_R)) + dJ*(b*(d*pd_dW)) - dJ*(omega_tensor(b*R,4)*pd_dW));

%% partial derivative with respect to dual ang. vel. vector
pd_dw = zeros(8,8);
pd_dw(1:3,1:3) = eye(3);
pd_dw(5:7,5:7) = eye(3);

d3 = dJ_inv*(-c*(dJ*pd_dw) + e*pd_dw - b*(dJ*pd_dw) + f*pd_dw - dJ*(b*pd_dw));

%% partial derivative with respect to dual thrust vector
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