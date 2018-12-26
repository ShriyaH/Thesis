function [A,b,G,h,C,dims] = SCvx_reevaluate_parameters(i,eta,x)
global CONSTANTS MAIN PARAMS Switch SC ITR Kleopatra;

% load parameters & compute constants
alpha0 =  CONSTANTS.alpha0;
T1 = CONSTANTS.T1;
T2 = CONSTANTS.T2;

% J = SC.I.I_total;
% r_T = [0;0;SC.dim.block(3)/3];
J= eye(3);
r_T = [-1;0;0];

t0 = CONSTANTS.t0;
tf = CONSTANTS.tf;
K = CONSTANTS.nodes;
ITR.t_k = (0:K-1)*tf/(K-1);
dt = (tf-t0)/K;

x0 = CONSTANTS.x0;
xf = CONSTANTS.xf;

m0 = CONSTANTS.x0(1);
mf = CONSTANTS.xf(1);
% m_wet = CONSTANTS.m_wet;
% m_dry = CONSTANTS.m_dry;

r0 = CONSTANTS.x0(2:4);
rf = CONSTANTS.xf(2:4);

v0 = CONSTANTS.x0(5:7);
vf = CONSTANTS.xf(5:7);

% g = [3 -1 -2]'; %considering constant gravitational field
% mu = Kleopatra.mu;
mu = 2;

PARAMS.n_state = 20;
PARAMS.n_control = 3;
PARAMS.n_virt = 3;

ns = PARAMS.n_state;
nc = PARAMS.n_control;
nv = PARAMS.n_virt;

w_vc = CONSTANTS.w_vc;
zeta_k = ones(K,1);

%% COST FUNCTION
% the formulation of the control vector is [X0 U0 X1 U1 X2 U2 ... XN UN]
X0 = zeros((ns+nc+nv)*K,1);

% construct the cost function - initialize it
C = zeros(size(X0));

% we want to minimize the final values of sigma
c_mod = reshape(C',ns+nc+nv,[]);
c_mod(1,:) = -1;
c_mod(ns+nc+1:ns+nc+nv,:) = w_vc;
C = c_mod(:);

% Dimensions of A and b
A = zeros((K+1)*ns,K*(ns+nc+nv));
b = zeros((K+1)*ns,1);

% Initial and final states
A(1:ns,1:ns) = eye(ns);  %Add matrix for x0
b(1:ns,1) = x0;
A(K*ns+1:(K+1)*ns,(K-1)*(ns+nc+nv)+1:(K-1)*(ns+nc+nv)+ns) = eye(ns);  %Add matrix for xf
b(K*ns+1:(K+1)*ns,1) = xf;

%% NEW LINEARISATION STATES AND CONTROLS

for k = 0:K-1
    x_k = x(:,k);
    m_k = x_k(1);
    r_k = x_k(2:4);
    v_k = x_k(5:7);
    q_k = x_k(8:11);
    w_k = x_k(12:14);
    T_k = x_k(15:17);
    Tdot_k = x_k(18:20);
    ITR.x_k{i+1}(1:20,k+1) = x_k;
    
    mdot_k = -alpha0*norm(T_k);
    rdot_k = v_k;
    vdot_k = (C_k*T_k)./m_k + (mu*r_k)/(norm(r_k))^3;
    qdot_k = zeros(4,1);
    wdot_k = inv(J)*(cross(r_T',T_k')' - cross(w_k, (J*w_k)));

    xdot_k = [mdot_k;rdot_k;vdot_k;qdot_k;wdot_k;Tdot_k;u_k];
    ITR.xdot_k{i+1}(1:20,k+1) = xdot_k;

%% DYNAMICS 

if k<K-1
    % construct continuous-time matrices at linearisation point
    Ac = zeros(ns,ns);
    Ac(1,15:17) = -alpha0.*(T_k'./norm(T_k));
    Ac(2:4,5:7) = eye(3);
    Ac(5:7,1) = (C_k*T_k)/m_k^2;
    Ac(5:7,2:4) = -(mu/(norm(r_k)^3))*eye(3);
    Ac(5:7,15:17) = C_k./m_k;
    Ac(8:11,8:11) = omega_tensor(w_k,2);
    Ac(12:14,12:14) = inv(J)*(omega_tensor(w_k,1)*J + omega_tensor(J*w_k,1));
    Ac(12:14,15:17) = inv(J)*omega_tensor(r_T,1);
    Ac(15:17,18:20) = eye(3);
    
    ITR.Ac_k{i+1}(ns*k+1:ns*(ii),1:ns) = Ac;
    
    Bc = zeros(ns,nc);
    Bc(ns-nc+1:ns-nc+3,:) = eye(nc);
    
    ITR.Bc_k{i+1}(ns*k+1:ns*(ii),1:nc) = Bc;
    
    zc = xdot_k - Ac*x_k - Bc*u_k;
    
    ITR.zc_k{i+1}(ns*k+1:ns*(ii),1) = zc;
     
    % DISCRETISATION
    
    % construct discrete-time matrices at linearisation points
    p = 2; %number of terms included in the series expansion 
    Psi = zeros(size(Ac));
    for pp = 0:1:p
        dPsi = (dt^pp/factorial(pp+1))*Ac^pp;
        Psi = Psi+dPsi;
    end
    
    Ad = eye(size(Ac)) + dt*Ac*Psi;
    ITR.Ad_k{i+1}(ns*k+1:ns*(ii),:) = Ad;
    
    Bd = dt*Psi*Bc;
    ITR.Bd_k{i+1}(ns*k+1:ns*(ii),:) = Bd;
    
    zd = dt*Psi*zc;
    ITR.zd_k{i+1}(ns*k+1:ns*(ii),:) = zd;
    
    % ECOS TRANSCRIPTION (Ax = b)
    
    jj = ii;
    idx = ii*ns+(1:ns);

    %state discrete matrices
    jdx = (jj-1)*(ns+nc+nv)+(1:ns);
    A(idx,jdx) = -Ad;
    b(idx,1) = zd;

    %control discrete matrices
    jdx = ns+(jj-1)*(ns+nc+nv)+(1:nc);
    A(idx,jdx) = -Bd;

    %virtual control discrete matrices
    jdx = ns+nc+(jj-1)*(ns+nc+nv)+(1:nv);
    Ec = zeros(ns,nc);
    Ec(ns-nc+1:ns-nc+3,:) = eye(nc);
    A(idx,jdx) = -Ec;
    
    if k<=K-2
    %next step state
    jdx = ns+nc+nv+(jj-1)*(ns+nc+nv)+(1:ns);
    A(idx,jdx) = eye(ns);
    end

end
     

%% CONSTRAINTS
G = [];
h = [];

if Switch.mass_lower_boundary_on % -m_k <= -mf 
    Gt = zeros(K,K*(ns+nc+nv));
    ht = zeros(K,1);
    for ii = 1:K-1
        Gt(ii,(ii-1)*(ns+nc+nv)+1) = -1;
        ht(ii) = -mf;
    end
    
    G = [G;Gt];
    h = [h;ht];
    dims.l = size(G,1);
end

if Switch.thrust_lower_boundary_on
    Gt = zeros(K,K*(ns+nc+nv));
    ht = zeros(K,1);
    
    for ii = 1:K
        T(:,ii) = ITR.x_k(15:17,ii);
        a(ii) = norm(T(:,ii));
        c(ii) = -T(:,ii)'*T(:,ii)./a(ii);
        ht(ii) = a(ii) + c(ii) - T1;
        
        Gt(ii,(ii-1)*(ns+nc+nv)+(15:17)) = -[1 1 1];
    end
    
    G = [G;Gt];
    h = [h;ht];
    dims.l =  size(G,1);
        
end

if Switch.thrust_upper_boundary_on
    Gt = zeros(K,K*(ns+nc+nv));
    ht = zeros(K,1);
    
    for ii = 1:K
        T(:,ii) = ITR.x_k(15:17,ii);
        a(ii) = norm(T(:,ii));
        c(ii) = -T(:,ii)'*T(:,ii)./a(ii);
        ht(ii) = -a(ii) - c(ii) + T2;
        
        Gt(ii,(ii-1)*(ns+nc+nv)+(15:17)) = [1 1 1];
    end
    
    G = [G;Gt];
    h = [h;ht];
    dims.l =  size(G,1);
        
end

dims.q = [];

if Switch.virtual_control_on % ||nu_k|| <= zeta_k 
    
   	S = zeros(3,K*(ns+nc+nv));
    s = zeros(1,K*(ns+nc+nv));
    bc = [0;0;0];
    
    for ii = 1:K
        Ac = S;
        Ac(1:3,(ii-1)*(ns+nc+nv)+(24:26)) = eye(3);
          
        cc = s;
        cc(1,(ii-1)*(ns+nc+nv)+(24:26)) = [0 0 0];
        
        dc = zeta_k(ii);
        
        Gt = [cc;Ac];
        ht = [dc;bc];
        
        G = [G;Gt];
        h = [h;ht];
    
        dims.q = [dims.q 4];
        
    end
end

if Switch.ang_rate_on % ||w_BI_k||_2 <= w_max
    w_max = CONSTANTS.w_max;
   	S = zeros(3,K*(ns+nc+nv));
    s = zeros(1,K*(ns+nc+nv));
    bc = [0;0;0];
    dc = w_max;
    for ii = 1:K
        Ac = S;
        Ac(1:3,(ii-1)*(ns+nc+nv)+(12:14)) = eye(3);
          
        cc = s;
        cc(1,(ii-1)*(ns+nc+nv)+(12:14)) = [0 0 0];
        
        Gt = [cc;Ac];
        ht = [dc;bc];
        
        G = [G;Gt];
        h = [h;ht];
    
        dims.q = [dims.q 4];
        
    end
end

if Switch.tilt_ang_on
    theta_tilt = CONSTANTS.theta_tilt;
    S = zeros(2,K*(ns+nc+nv));
    s = zeros(1,K*(ns+nc+nv));
    bc = [0;0];
    dc = sqrt((1-cos(theta_tilt))/2);
    for ii = 1:K
        Ac = S;
        Ac(1,(ii-1)*(ns+nc+nv)+(8:11)) = [0 1 0 0];
        Ac(2,(ii-1)*(ns+nc+nv)+(8:11)) = [0 0 1 0];
          
        cc = s;
        cc(1,(ii-1)*(ns+nc+nv)+(8:11)) = [0 0 0 0];
        
        Gt = [cc;Ac];
        ht = [dc;bc];
        
        G = [G;Gt];
        h = [h;ht];
    
        dims.q = [dims.q 3];
        
    end
    
end

if Switch.gimbal_ang_on
    theta_gm = CONSTANTS.theta_gm;
    S = zeros(3,K*(ns+nc+nv));
    s = zeros(1,K*(ns+nc+nv));
    bc = [0;0;0];
    dc = 0;
    for ii = 1:K
        Ac = S;
        Ac(1:3,(ii-1)*(ns+nc+nv)+(15:17)) = eye(3);
          
        cc = s;
        cc(1,(ii-1)*(ns+nc+nv)+(15:17)) = [1/cos(theta_gm) 0 0];
        
        Gt = [cc;Ac];
        ht = [dc;bc];
        
        G = [G;Gt];
        h = [h;ht];
    
        dims.q = [dims.q 4];
        
    end
end

if Switch.glideslope_on
    theta_gs = CONSTANTS.theta_gs;
    S = zeros(2,K*(ns+nc+nv));
    s = zeros(1,K*(ns+nc+nv));
    bc = [0;0];
    dc = 0;
    for ii = 1:K
        Ac = S;
        Ac(1,(ii-1)*(ns+nc+nv)+(2:4)) = [tan(theta_gs) 0 0];
        Ac(2,(ii-1)*(ns+nc+nv)+(2:4)) = [0 tan(theta_gs) 0];
        
        cc = s;
        cc(1,(ii-1)*(ns+nc+nv)+(2:4)) = [0 0 1];
        
        Gt = -[cc;Ac];
        ht = [dc;bc];
        
        G = [G;Gt];
        h = [h;ht];
    
        dims.q = [dims.q 3];
        
    end
end

G = sparse(G);
A = sparse(A);
end
