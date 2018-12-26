function [] = first_sol2()
% Asteroid descent problem for ECOS with Successive Convexification
% Shriya Hazra, 31-Jul-2018 
global ITR CONSTANTS;

K = CONSTANTS.nodes;
alpha0 =  CONSTANTS.alpha0;
T1 = CONSTANTS.T1;
T2 = CONSTANTS.T2;

J = CONSTANTS.J;
r_T = CONSTANTS.r_T;

t0 = CONSTANTS.t0;
tf = CONSTANTS.tf;
K = CONSTANTS.nodes;
ITR.t_k = (0:K-1)*tf/(K-1);
dt = (tf-t0)/K;

x0 = CONSTANTS.x0;
xf = CONSTANTS.xf;
m0 = CONSTANTS.x0(1);
mf = CONSTANTS.xf(1);

r0 = CONSTANTS.x0(2:4);
rf = CONSTANTS.xf(2:4);

v0 = CONSTANTS.x0(5:7);
vf = CONSTANTS.xf(5:7);

g = CONSTANTS.g; 


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
    v_k = ((K-k-1)/(K-1))*v0 + (k/(K-1))*vf;
    q_k = [0 0 0 1]';
    w_k = zeros(3,1);
    C_k =  Q2DCM(conj_quat(q_k));  
    T_k = -(m_k*quat_trans(q_k,g,'vect'))';
    Tdot_k = zeros(3,1);
    
    x_k  = [m_k;r_k;v_k;q_k;w_k;T_k;Tdot_k];
    ITR.x_k{1}(:,ii) = x_k;
    
    u_k = zeros(3,1);
    ITR.u_k{1}(:,ii) = u_k;
        
    if k<K-1        
        % construct linearisation point state differential
        mdot_k = -alpha0*norm(T_k);
        rdot_k = v_k;
        vdot_k = (C_k*T_k)./m_k + g';
        qdot_k = 1/2.*(omega_tensor(w_k,2)*q_k);
        wdot_k = inv(J)*(cross(r_T',T_k')' - cross(w_k, (J*w_k)));

        xdot_k = [mdot_k;rdot_k;vdot_k;qdot_k;wdot_k;Tdot_k;u_k];
        ITR.xdot_k{1}(:,ii) = xdot_k;
    end
end
ITR.m_spent(1,1) = ITR.x_k{1}(1,1) - ITR.x_k{1}(1,end);
end