function [] = first_sol_DQ()
% Asteroid descent problem for ECOS with Successive Convexification
% Shriya Hazra, 31-Jul-2018 
global ITR CONSTANTS Switch Kleopatra;

K = CONSTANTS.nodes;
alpha0 =  CONSTANTS.alpha0;

J = CONSTANTS.J;
r_F = CONSTANTS.r_F;

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

q0 = CONSTANTS.x0(2:9);
qf = CONSTANTS.xf(2:9);

w0 = CONSTANTS.x0(10:17);
wf = CONSTANTS.xf(10:17);

g = CONSTANTS.g; 
w_ai = CONSTANTS.w_AI;
q = ScLERP(q0, qf, ITR.t_k);

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
    q_k = q(:,ii);
    w_k = ((K-k-1)/(K-1))*w0 + (k/(K-1))*wf;
    F = -(m_k*quat_trans(q_k(1:4),g,'vect'))';
    F_k = [F;0;cross(r_F',F')';0];
    Fdot_k = zeros(8,1);
    
    if Switch.constant_grav_on==1
        F_GG_k = [m_k.*g;0;0;0;0;0];
    else
        [g_B, T_GG_B] = Get_grav(q_k,Kleopatra);
        F_GG_k = [m_k.*g_B;0;T_GG_B;0];
    end
    
    wa = quat_trans(q_k(1:4),w_ai,'n');
    wa = [wa;0;0;0;0];
    Wa = omega_tensor(wa,4);
    
    R = [0;0;0;0;2.*cross_quat(q_k(5:8),conj_quat(q_k(1:4)))];
    
    x_k  = [m_k;q_k;w_k;F_k;Fdot_k];
    ITR.x_k{1}(:,ii) = x_k;
    
    u_k = zeros(8,1);
    ITR.u_k{1}(:,ii) = u_k;
        
    if k<K-1       
        J_k = dq_inertia(m_k,J);
        [U, S, V] = svd(J_k);
        J_inv_k = V*(S\U');
        
        % construct linearisation point state differential
        mdot_k = -alpha0*norm(F_k(1:4));
        qdot_k = 1/2.*(omega_tensor(w_k,3)*q_k);
        wdot_k = J_inv_k*(F_k + F_GG_k - omega_tensor((w_k+wa),4)*(J_k*(w_k+wa)) - J_k*(Wa*w_k) - J_k*(Wa*(Wa*R)));

        xdot_k = [mdot_k;qdot_k;wdot_k;Fdot_k;u_k];
        ITR.xdot_k{1}(:,ii) = xdot_k;
    end
end
ITR.m_spent(1,1) = ITR.x_k{1}(1,1) - ITR.x_k{1}(1,end);
end