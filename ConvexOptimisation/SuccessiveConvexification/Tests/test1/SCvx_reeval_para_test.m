function [A,b,G,h,C,dims] = SCvx_reeval_para_test(i,ek)
global CONSTANTS PARAMS Switch ITR ;

% load parameters & compute constants
alpha0 =  CONSTANTS.alpha0;
T1 = CONSTANTS.T1;
T2 = CONSTANTS.T2;

J = CONSTANTS.J;
r_T = CONSTANTS.r_T;

t0 = CONSTANTS.t0;
tf = CONSTANTS.tf;
K = CONSTANTS.nodes;
dt = (tf-t0)/K;

x0 = CONSTANTS.x0;
xf = CONSTANTS.xf;

m0 = CONSTANTS.x0(1);
mf = CONSTANTS.xf(1);

r0 = CONSTANTS.x0(2:4);
rf = CONSTANTS.xf(2:4);

v0 = CONSTANTS.x0(5:7);
vf = CONSTANTS.xf(5:7);

g = CONSTANTS.g; %considering constant gravitational field

w_vc = CONSTANTS.w_vc;
% w_tr = CONSTANTS.w_tr;
eta_k = ITR.eta_k{ek+1};

%Initialize state and control sizes
PARAMS.n_state = 20;
PARAMS.n_control = 3;
PARAMS.n_virt = 20;
PARAMS.n_slack = 1;

ns = PARAMS.n_state;
nc = PARAMS.n_control;
nv = PARAMS.n_virt;
nsl = PARAMS.n_slack;
n = ns+nc+nv+nsl;

% % define the final elements we impose
X_f_flag = ones(ns,1);
n_flag = sum(X_f_flag);

%% COST FUNCTION
% the formulation of the control vector is [X0 U0 nu0 s0 X1 U1 nu1 s1... XN UN nuN sN]
X0 = zeros(n*K,1);

% construct the cost function - initialize it
C = zeros(size(X0));

% we want to minimize the final value of mass and the penalised virtual controls and trust regions 
c_mod = reshape(C',n,[]);
c_mod(ns+nc+nv+1:n,:) = w_vc;
C = c_mod(:);
C(n*(K-1)+1,1) = -1;


%% DYNAMICS
% Dimensions of A and b
A = zeros((K+1)*ns,K*n);
b = zeros((K+1)*ns,1);

% Identity matrices for state vector at current time step 
for ii = 1:K
    idx = (ii-1)*ns+(1:ns);
    jdx = (ii-1)*(n)+(1:ns);
    A(idx,jdx) = eye(ns);  
end
A(K*ns+(1:ns),(K-1)*(n)+(1:ns)) = eye(ns); 
b(1:ns,1) = x0; %initial state
b(K*ns+(1:ns),1) = xf; %final state

% Intialise array structure for all iterate states, controls and matrices
Ed = zeros(ns,nv+nsl);
Ed(1:nv,1:nv) = eye(nv);

%% NEW LINEARISATION STATES AND CONTROLS
for k = 1:K
    % construct linearisation point states
    x_k = ITR.x_k{i+1}(:,k);
    m_k = x_k(1,1);
    q_k = x_k(8:11,1);
    w_k = x_k(12:14,1); 
    T_k = x_k(15:17,1);
   
    if k<K
        
        % construct linearisation point state differential
        xdot_k = ITR.xdot_k{i+1}(:,k);
        u_k = ITR.u_k{i+1}(:,k);

        % construct continuous-time matrices at linearisation point
        Ac = zeros(ns,ns);
        Ac(1,15:17) = -alpha0.*(T_k'./norm(T_k));
        Ac(2:4,5:7) = eye(3);
        Ac(5:7,:) = get_da(T_k,q_k,m_k,ns);
        Ac(8:11,:) = get_dqdot(w_k,q_k,ns); 
        Ac(12:14,:) = get_dwdot(w_k,J,r_T,ns);
        Ac(15:17,18:20) = eye(3);


        ITR.Ac_k{i+1}{k} = Ac;

        Bc = zeros(ns,nc);
        Bc(18:20,:) = eye(nc);

        ITR.Bc_k{i+1}{k} = Bc;

        zc = xdot_k - Ac*x_k - Bc*u_k;

        ITR.zc_k{i+1}{k} = zc;

    % Discretisation
        % state transition matrix for each time step
        p = 3; %number of terms included in the series expansion 
        Psi = zeros(size(Ac));
        for pp = 0:1:p
            dPsi = (dt^pp/factorial(pp+1))*Ac^pp;
            Psi = Psi+dPsi;
        end

        % construct discrete-time matrices at linearisation points
        Ad = eye(size(Ac)) + dt.*(Ac*Psi);
        ITR.Ad_k{i+1}{k} = Ad;

        Bd = dt.*(Psi*Bc);
        ITR.Bd_k{i+1}{k} = Bd;

        zd = dt.*(Psi*zc);
        ITR.zd_k{i+1}{k} = zd;

        % discrete matrices in A matrix  
        jj = k;
        idx = k*ns+(1:ns);

        % state discrete matrices
        jdx = (jj-1)*n+(1:ns);
        A(idx,jdx) = -Ad;

        % control discrete matrices
        jdx = ns+(jj-1)*(n)+(1:nc);
        A(idx,jdx) = -Bd;

        % virtual control discrete matrices
        jdx = ns+nc+(jj-1)*(n)+(1:nv+nsl);
        A(idx,jdx) = -Ed;

        % construct b
        b(idx,1) = zd;
    end   
end

if Switch.discrete_higherorder_on
        phi = {};
        for ii = 1:K-2
            phi{ii}{1} = ITR.Ad_k{1}{ii+1};
            for jj = 1:(K-2-ii)
                phi{ii}{jj+1} = ITR.Ad_k{1}{ii+jj+1}*phi{ii}{jj};
            end
        end

        for ii = 0:K
            jdx = ii*n + ns + (1:nc);    
            for jj = 1:(K-3-ii) 
            idx = (1+jj+ii)*ns + (1:ns);    
            A(idx,jdx) = -phi{ii+1}{jj}*ITR.Bd_k{1}{ii+1};
            end
        end

        Z = zeros(ns*K,ns*(K-1));
        Z(ns+1:end,1:end) = eye(ns*(K-1)); 
        for ii = 0:K-2
            jdx = ii*ns + (1:ns);    
            for jj = 1:(K-2-ii) 
            idx = (1+jj+ii)*ns + (1:ns);
            Z(idx,jdx)= phi{ii+1}{jj};
            end
            idx = ii*ns +(1:ns);
            z(idx,1) = ITR.zd_k{1}{ii+1};
        end
        
        b1 = Z*z;
        b = zeros(K*ns,1);
        b(1:ns,1) = x0; %initial state
        b = b + b1;
end  
    

%% CONSTRAINTS
G = [];
h = [];

if Switch.mass_lower_boundary_on
    Gt = zeros(K,K*n);
    ht = zeros(K,1);
    
    for ii = 1:K
        ht(ii) = -mf;
        
        Gt(ii,(ii-1)*n+1) = -1;
    end
    
    G = [G;Gt];
    h = [h;ht];
    dims.l =  size(G,1);
        
end


if Switch.thrust_lower_boundary_on 
    Gt = zeros(K,K*n);
    ht = zeros(K,1);
    
    for ii = 1:K
        T(:,ii) = ITR.x_k{1}(15:17,ii);
        T_norm(ii) = norm(T(:,ii));
        T_norma(ii,:) = T(:,ii)'./T_norm(ii);
        T_dot(ii) = -T_norma(ii,:)*T(:,ii);
        ht(ii) = T_norm(ii) + T_dot(ii) - T1;
        
        Gt(ii,(ii-1)*n+(15:17)) = -T_norma(ii);
    end
    
    G = [G;Gt];
    h = [h;ht];
    dims.l =  size(G,1);
        
end

if Switch.thrust_upper_boundary_on
    Gt = zeros(K,K*n);
    ht = zeros(K,1);
    
    for ii = 1:K
        T(:,ii) = ITR.x_k{1}(15:17,ii);
        T_norm(ii) = norm(T(:,ii));
        T_norma(ii,:) = T(:,ii)'./T_norm(ii);
        T_dot(ii) = -T_norma(ii,:)*T(:,ii);
        ht(ii) = T2 - T_norm(ii) - T_dot(ii);
        
        Gt(ii,(ii-1)*n+(15:17)) = T_norma(ii);
    end
    
    G = [G;Gt];
    h = [h;ht];
    dims.l =  size(G,1);
        
end

% if Switch.virtual_control_on % s_k <= zeta_k
%         Gt = zeros(K,K*n);
%         ht = zeros(K,1);
% 
%         for ii = 1:K        
%             Gt(ii,(ii-1)*n+ns+nc+nv+(1:nsl)) = 1;
%             ht(ii,1) = zeta_k(ii);
%         end
% 
%         G = [G;Gt];
%         h = [h;ht];
%         dims.l =  size(G,1);
% 
% end

dims.q = [];

% %quaternion norm %||q_k||_2 = 1
% S = zeros(4,K*n);
% s = zeros(1,K*n);
% bc = zeros(4,1);
%     
% for ii = 1:K
%     Ac = S;
%     Ac(1:4,(ii-1)*n+(8:11)) = eye(4);
% 
%     cc = s;
% 
%     dc = 1;
%     Gt = -[cc;Ac];
%     ht = [dc;bc];
% 
%     G = [G;Gt];
%     h = [h;ht];
% 
%     dims.q = [dims.q 5];
%         
% end


% if Switch.mass_lower_boundary_on % ||mf||_2 <= m_k
%     S = zeros(1,K*n);
%     s = zeros(1,K*n);
%     bc = mf;
%     dc = 0;
%     for ii = 1:K
%         Ac = S;
%           
%         cc = s;
%         cc(1,(ii-1)*n+1) = 1;
%         
%         Gt = -[cc;Ac];
%         ht = [dc;bc];
%         
%         G = [G;Gt];
%         h = [h;ht];
%     
%         dims.q = [dims.q 2];
%         
%     end
% end

if Switch.ang_rate_on % ||w_BI_k||_2 <= w_max
    w_max = CONSTANTS.w_max;
   	S = zeros(3,K*n);
    s = zeros(1,K*n);
    bc = [0;0;0];
    dc = w_max;
    for ii = 1:K
        Ac = S;
        Ac(1:3,(ii-1)*n+(12:14)) = eye(3);
          
        cc = s;
        
        Gt = -[cc;Ac];
        ht = [dc;bc];
        
        G = [G;Gt];
        h = [h;ht];
    
        dims.q = [dims.q 4];
        
    end
end


if Switch.gimbal_ang_on
    theta_gm = CONSTANTS.theta_gm;
    S = zeros(3,K*n);
    s = zeros(1,K*n);
    bc = [0;0;0];
    dc = 0;
    for ii = 1:K
        Ac = S;
        Ac(1:3,(ii-1)*n+(15:17)) = cos(theta_gm).*eye(3);
          
        cc = s;
        cc(1,(ii-1)*n+(15:17)) = [1 0 0];
        
        Gt = -[cc;Ac];
        ht = [dc;bc];
        
        G = [G;Gt];
        h = [h;ht];
    
        dims.q = [dims.q 4];
        
    end
end

if Switch.glideslope_on
    theta_gs = CONSTANTS.theta_gs;
    S = zeros(2,K*n);
    s = zeros(1,K*n);
    bc = [0;0];
    dc = 0;
    for ii = 1:K
        Ac = S;
        Ac(1,(ii-1)*n+(2:4)) = [0 tan(theta_gs) 0];
        Ac(2,(ii-1)*n+(2:4)) = [0 0 tan(theta_gs)];
        
        cc = s;
        cc(1,(ii-1)*n+(2:4)) = [1 0 0];
        
        Gt = -[cc;Ac];
        ht = [dc;bc];
        
        G = [G;Gt];
        h = [h;ht];
    
        dims.q = [dims.q 3];
        
    end
end

if Switch.virtual_control_on %||v_k||_2 <= s_k
    S = zeros(nv,K*n);
    s = zeros(1,K*n);
    bc = zeros(nv,1);
    
    for ii = 1:K
        Ac = S;
        Ac(1:nv,(ii-1)*n+ns+nc+(1:nv)) = eye(nv);
          
        cc = s;
        cc(1,(ii-1)*n+ns+nc+nv+(1:nsl)) = 1;
        
        dc = 0;
        Gt = -[cc;Ac];
        ht = [dc;bc];
        
        G = [G;Gt];
        h = [h;ht];
    
        dims.q = [dims.q (nv+1)];
        
    end
end

if Switch.trust_region_on % ||(1 - 2x_k^Tx + (x_k^Tx_k - eta_k))/2; Ax|| <= (1 + 2x_k^Tx - (x_k^Tx_k - eta_k))/2
    
   	S = zeros(ns+nc+1,K*n);
    s = zeros(1,K*n);
    bc = zeros(ns+nc+1,1);  
    for ii = 1:K
        xk(:,ii) = [ITR.x_k{1}(:,ii);ITR.u_k{1}(:,ii)];
        c(ii) = (xk(:,ii)'*xk(:,ii) - eta_k(ii))/2;
        
        Ac = S;
        Ac(1,(ii-1)*n+(1:(ns+nc))) = -xk(:,ii)';
        Ac(2:(ns+nc+1),(ii-1)*n+(1:(ns+nc))) = eye(ns+nc);
        
        cc = s;
        cc(1,(ii-1)*n+(1:(ns+nc))) = xk(:,ii)';
        
        dc = (1 - xk(:,ii)'*xk(:,ii) + eta_k(ii))/2;
                 
        bc(1,1) = 0.5 + c(ii);
        
        Gt = -[cc;Ac];
        ht = [dc;bc];
        
        G = [G;Gt];
        h = [h;ht];
    
        dims.q = [dims.q ns+nc+2];
        
    end
end   

G = sparse(G);
A = sparse(A);
end