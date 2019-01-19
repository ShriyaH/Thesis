function [] = first_sol(Asteroid)
% Asteroid descent problem for ECOS with Successive Convexification
% Shriya Hazra, 31-Jul-2018 
global ITR CONSTANTS Switch;

K = CONSTANTS.nodes;
alpha0 =  CONSTANTS.alpha0;

J = CONSTANTS.J;
r_F = CONSTANTS.r_F;
dq_form = CONSTANTS.dq_form;

t0 = CONSTANTS.t0;
tf = CONSTANTS.tf;
K = CONSTANTS.nodes;
ITR.t_k = (0:K-1)*tf/(K-1);
dt = (tf-t0)/K;
CONSTANTS.dt = dt;

x0 = CONSTANTS.x0;
xf = CONSTANTS.xf;
m0 = CONSTANTS.x0(1);
mf = CONSTANTS.xf(1);

dq0 = CONSTANTS.x0(2:9);
dqf = CONSTANTS.xf(2:9);

w0 = CONSTANTS.x0(10:12);
wf = CONSTANTS.xf(10:12);

v0 = quat_trans(conj_quat(dq0(1:4)),CONSTANTS.x0(14:17),'vect')';
vf = quat_trans(conj_quat(dq0(1:4)),CONSTANTS.xf(14:17),'vect')'; 

r0 = DQ2R(dq0,dq_form);
rf = DQ2R(dqf,dq_form);

dwa = CONSTANTS.dw_AI;
q = ScLERP(dq0, dqf, ITR.t_k);

ITR.x_k ={};
ITR.u_k ={};
ITR.xdot_k = {};
ITR.Ac_k = {};
ITR.Bc_k = {};
ITR.zc_k = {};
ITR.Ad_k = {};
ITR.Bd_k = {};
ITR.zd_k = {};
ITR.eta_k = {};


for k = 0:K-1
    ii = k+1;  
    
    % construct linearisation point states
    m_k = ((K-k-1)/(K-1))*m0 + (k/(K-1))*mf;
    r_k = ((K-k-1)/(K-1))*r0 + (k/(K-1))*rf;
    dq_k = Q2DQ(dq0(1:4),r_k,dq_form);
    v_k = ((K-k-1)/(K-1))*v0 + (k/(K-1))*vf;
    v_k = quat_trans(dq_k(1:4),v_k,'n');
    w_k = ((K-k-1)/(K-1))*w0 + (k/(K-1))*wf;
    dw_k = [w_k;0;v_k];
    
    if Switch.constant_grav==1
        g = CONSTANTS.g; 
        gb = quat_trans(dq_k(1:4),g,'vect')';
        dFg_k = [m_k.*gb;0;0;0;0;0];
    else
        [gb, Tgb] = Get_pertforces(C_BA,r_B,rs_B,e_B,mu,Asteroid);
        dFg_k = [m_k.*gb;0;Tgb;0];
    end
    
    F = -(m_k.*gb);
    dF_k = [F;0;cross(r_F',F')';0];
    Fdot_k = -alpha0.*norm(dF_k(1:4)).*gb;
    dFdot_k = [Fdot_k;0;cross(r_F',Fdot_k')';0];
    
    wab = dq_trans(dq_k,dwa);
    Wab = omega_tensor(wab,4);
    
    R = [0;0;0;0;DQ2R(dq_k,dq_form)];
    
    x_k  = [m_k;dq_k;dw_k;dF_k;dFdot_k];
    ITR.x_k{1}(:,ii) = x_k;
    
    u_k = zeros(8,1);
    ITR.u_k{1}(:,ii) = u_k;
        
    if k<K-1       
        J_k = dq_inertia(m_k,J);
        [U, S, V] = svd(J_k);
        J_inv_k = V*(S\U');
        
        % construct linearisation point state differential
        mdot_k = -alpha0*norm(dF_k(1:4));
        dqdot_k = 1/2.*(omega_tensor(dw_k,3)*dq_k);
        dwdot_k = J_inv_k*(dF_k + dFg_k - omega_tensor((dw_k+wab),4)*(J_k*(dw_k+wab)) - J_k*(Wab*dw_k) - J_k*(Wab*(Wab*R)));

        xdot_k = [mdot_k;dqdot_k;dwdot_k;dFdot_k;u_k];
        ITR.xdot_k{1}(:,ii) = xdot_k;
    end
end

ITR.m_spent(1,1) = ITR.x_k{1}(1,1) - ITR.x_k{1}(1,end);
end