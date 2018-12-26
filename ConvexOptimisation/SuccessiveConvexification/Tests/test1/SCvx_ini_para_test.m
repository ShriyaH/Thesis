function [A,b,G,h,C,dims] = SCvx_ini_para_test(i)

% Standard formulation
global CONSTANTS PARAMS Switch ITR;

% load parameters & compute constants
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

mf = CONSTANTS.xf(1);

%considering constant gravitational field

w_vc = CONSTANTS.w_vc;
% w_tr = CONSTANTS.w_tr;

%Initialize state and control sizes
ns = PARAMS.n_state;
nc = PARAMS.n_control;
nv = PARAMS.n_virt;
nsl = PARAMS.n_slack;
n = ns+nc+nv+nsl;

Ed = zeros(ns,nv+nsl);
Ed(1:nv,1:nv) = eye(nv);

% define the final elements we impose
X_f_flag = [0;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1;1];
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
C(end,1) = 0;

%% DYNAMICS
% Dimensions of A and b
A = zeros(K*ns+n_flag,K*n);
b = zeros(K*ns+n_flag,1);

% Identity matrices for state vector at current time step 
for ii = 1:K
    idx = (ii-1)*ns+(1:ns);
    jdx = (ii-1)*(n)+(1:ns);
    A(idx,jdx) = eye(ns);  
end
b(1:ns,1) = x0; %initial state

A(K*ns+(1:n_flag),(K-1)*(n)+1+(1:n_flag)) = eye(n_flag); 
b(K*ns+(1:n_flag),1) = xf(2:ns,1); %final state


% Construct all the LTV continuous time matrices for initial iteration
% state (K-1 time steps)
for k = 0:K-1
    ii = k+1;
    % construct linearisation point states
    x_k = ITR.x_k{i}(:,ii);
    m_k = x_k(1,1);
    q_k = x_k(8:11,1);
    w_k = x_k(12:14,1); 
    T_k = x_k(15:17,1);
   
    if ii<K
        
        % construct linearisation point state differential
        xdot_k = ITR.xdot_k{i}(:,ii);
        u_k = ITR.u_k{i}(:,ii);
    
        % construct continuous-time matrices at linearisation point
        Ac = zeros(ns,ns);
        Ac(1,15:17) = -alpha0.*(T_k'./norm(T_k));
        Ac(2:4,5:7) = eye(3);
        Ac(5:7,:) = get_da(T_k,q_k,m_k,ns);
        Ac(8:11,:) = get_dqdot(w_k,q_k,ns); 
        Ac(12:14,:) = get_dwdot(w_k,J,r_T,ns);
        Ac(15:17,18:20) = eye(3);


        ITR.Ac_k{i}{ii} = Ac;

        Bc = zeros(ns,nc);
        Bc(18:20,:) = eye(nc);

        ITR.Bc_k{i}{ii} = Bc;

        zc = xdot_k - Ac*x_k - Bc*u_k;

        ITR.zc_k{i}{ii} = zc;

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
        ITR.Ad_k{i}{ii} = Ad;

        Bd = dt.*(Psi*Bc);
        ITR.Bd_k{i}{ii} = Bd;

        zd = dt.*(Psi*zc);
        ITR.zd_k{i}{ii} = zd;

        % discrete matrices in A matrix  
        jj = ii;
        idx = ii*ns+(1:ns);

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

%Quaternion norm constraint
if Switch.quat_bound
    Aq = zeros(K,K*n);
    bq = zeros(K,1);
    for ii = 1:K
        jdx = (ii-1)*n+(8:11);
        idx = ii;
        Aq(idx,jdx)=ITR.x_k{i}(8:11,ii)';
        bq(idx,1)=1;
    end
    A = [A;Aq];
    b = [b;bq];
end

if Switch.discrete_higherorder_on
          phi = {};
  for k = 1:K-1
       for ii = 1:K-k
           if ii == 1 
               phi{k}{ii} = ITR.Ad_k{1}{k}; 
           else
               phi{k}{ii} = ITR.Ad_k{1}{k}*phi{k}{ii-1};
           end
       end
   end
           
   for ii = 1:K-2
       jdx = (ii-1)*n + ns + (1:nc);    
       for jj = 1:(K-1-ii) 
       idx = (jj+ii)*ns + (1:ns);    
       A(idx,jdx) = -phi{ii+1}{jj}*ITR.Bd_k{i}{ii};   
       end
   end

   Z = zeros(ns*K,ns*(K-1));
   for ii = 1:K-1
       idx = (ii)*ns + (1:ns);
       jdx = (ii-1)*ns + (1:ns);
       Z(idx,jdx) = eye(ns); 

       idx = (ii-1)*ns +(1:ns);
       z(idx,1) = ITR.zd_k{i}{ii};
   end

   for ii = 1:K-2
       jdx = (ii-1)*ns + (1:ns);    
       for jj = 1:(K-1-ii) 
       idx = (jj+ii)*ns + (1:ns);    
       Z(idx,jdx) = phi{ii+1}{jj};   
       end
   end

z_new = Z*z;
z_new = z_new(ns+1:end,1);

b(ns+1:K*ns,1) = z_new;  
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

if Switch.quat_bound
    Gt = zeros(4*K,K*n);
    ht = ones(4*K,1);
   
    for ii = 1:K
        Gt(4*ii-3:4*ii,(ii-1)*n + (8:11)) = eye(4);
    end
    
    G = [G;Gt];
    h = [h;ht];
    dims.l =  size(G,1);
end

if Switch.thrust_lower_boundary_on 
    Gt = zeros(K,K*n);
    ht = zeros(K,1);
    
    for ii = 1:K
        T(:,ii) = ITR.x_k{i}(15:17,ii);
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
        T(:,ii) = ITR.x_k{i}(15:17,ii);
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
        Ac(1:3,(ii-1)*n+(15:17)) = eye(3);
          
        cc = s;
        cc(1,(ii-1)*n+(15:17)) = [1/cos(theta_gm) 0 0];
        
        Gt = -[cc;Ac];
        ht = [dc;bc];
        
        G = [G;Gt];
        h = [h;ht];
    
        dims.q = [dims.q 4];
        
    end
end

if Switch.tilt_ang_on
    theta_tilt = CONSTANTS.theta_tilt;
    S = zeros(2,K*n);
    s = zeros(1,K*n);
    bc = [0;0];
    dc = sqrt((1-cos(theta_tilt))/2);
    for ii = 1:K
        Ac = S;
        Ac(1,(ii-1)*n+(8:11)) = [0 1 0 0];
        Ac(2,(ii-1)*n+(8:11)) = [0 0 1 0];
          
        cc = s;
        
        Gt = -[cc;Ac];
        ht = [dc;bc];
        
        G = [G;Gt];
        h = [h;ht];
    
        dims.q = [dims.q 3];
        
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
        Ac(1,(ii-1)*n+(2:4)) = [0 1 0];
        Ac(2,(ii-1)*n+(2:4)) = [0 0 1];
        
        cc = s;
        cc(1,(ii-1)*n+(2:4)) = [1/tan(theta_gs) 0 0];
        
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
        xk(:,ii) = [ITR.x_k{i}(:,ii);ITR.u_k{i}(:,ii)];
        c(ii) = (xk(:,ii)'*xk(:,ii) - ITR.eta_k{i}(ii))/2;
        
        Ac = S;
        Ac(1,(ii-1)*n+(1:(ns+nc))) = -xk(:,ii)';
        Ac(2:(ns+nc+1),(ii-1)*n+(1:(ns+nc))) = eye(ns+nc);
        
        cc = s;
        cc(1,(ii-1)*n+(1:(ns+nc))) = xk(:,ii)';
        
        dc = 0.5 - c(ii);
                 
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
