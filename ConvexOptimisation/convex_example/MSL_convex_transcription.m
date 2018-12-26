function[tc,xc,uc,xdotc,cpu_time,status] = MSL_convex_transcription(x0,xf,n,tf)
% coding the Acikmese problem for ECOS with Acikmese's rules
%
% 
%
% Marco Sagliano, DLR, 05-Jan-2017, 
% copyright DLR

% Standard formulation

global CONSTANTS MAIN PARAMS;

% linear constraints
upper_boundary_sigma_on = 1;
no_sub_surface_on = 1;
limits_z_on = 1;

% conic constraints
lower_boundary_sigma_on = 1;
glideslope_on = 1;
limits_control_on = 1;

% load parameters & compute constants
MSL_descent_main;
alpha =  CONSTANTS.sigma;
rho1 = 0.3*CONSTANTS.T_max*6*cos(27*pi/180);
rho2 = 0.8*CONSTANTS.T_max*6*cos(27*pi/180);

% define the final elements we impose
X_f_flag = [1; 1; 1; 1; 1; 1; 0];
n_flag = sum(X_f_flag);

% define states and controls
PARAMS.n_states = 7;
PARAMS.n_controls = 4;
ns = PARAMS.n_states;
nc = PARAMS.n_controls;

% init ECOS inputs
A = [];
b = [];
G = [];
h = [];
dims = [];

% construct time vector
t0 =  0;
t = linspace(t0,tf,n);

% define stepsize
dt = (tf-t0)/(n-1);
tc = t;

% compute boundaries for z
z_l = log(CONSTANTS.m0-alpha*rho2*t);
z_u = log(CONSTANTS.m0-alpha*rho1*t);

% compute mu1 and mu2
mu1 = rho1.*exp(-z_l);
mu2 = rho2.*exp(-z_u);

% the formulation of the control vector is [X1 U1 X2 U2 ... XN UN]
X0 = zeros(ns*n+nc*n,1);

% construct the cost function - initialize it
C = zeros(size(X0));

% we want to minimize the final values of sigma
c_mod = reshape(C',ns+nc,[]);
c_mod(end,:) = dt;
C = c_mod(:);

% construct the matrix A and b

% LTI continuous matrices are Ac and Bc
Ac = zeros(7,7); % x,y,z,xdot,ydot,zdot,mdot
Ac(1:3,4:6) = eye(3);
Bc = zeros(7,4); % u1,u2,u3,sigma
Bc(4:6,1:3) = eye(3);
Bc(7,4) = -alpha;
Ac_lti = Ac;
Bc_lti = Bc;
% conversion to discrete - first, compute Psi
p = 2; % terms included in p

Psi = zeros(size(Ac));
for ii = 0:1:p
    dPsi = dt^ii/(factorial(ii+1))*Ac^ii;
    Psi = Psi+dPsi;
end

% conversion to discrete matrices
Ad = eye(7)+dt*Ac*Psi;
Bd = dt*Psi*Bc;

% what are the dimensions of A and b?
A = zeros(n*PARAMS.n_states+n_flag,n*(PARAMS.n_states+PARAMS.n_controls));
b = zeros(n*PARAMS.n_states+n_flag,1);


% TRANSCRIPTION - CONVEX

%%%% INITIAL CONDITIONS
b(1:PARAMS.n_states,1) = x0;
A(1:PARAMS.n_states,1:PARAMS.n_states) = eye(PARAMS.n_states);

%%%% DYNAMICS
% we have to impose in n-1 nodes the discrete-system differential equations
for ii = 1:n-1
    idx = (ii)*PARAMS.n_states+(1:PARAMS.n_states);
    jj = ii;
    
    % state-related part - diagonal
    jdx = (jj-1)*(PARAMS.n_states+PARAMS.n_controls)+(1:PARAMS.n_states);
    A(idx,jdx) = -Ad;
    % control-related part - diagonal
    jdx = PARAMS.n_states+(jj-1)*(PARAMS.n_states+PARAMS.n_controls)+(1:PARAMS.n_controls);
    A(idx,jdx) = -Bd;
    b(idx,1) = Bd*[0; 0; -CONSTANTS.g0; 0];
    % state-related part - diagonal
    jdx = (jj)*(PARAMS.n_states+PARAMS.n_controls)+(1:PARAMS.n_states);
    A(idx,jdx) = eye(ns);
    
end

%%%% FINAL CONDITIONS
A(n*PARAMS.n_states+(1:n_flag),(n-1)*(PARAMS.n_states+PARAMS.n_controls)+(1:n_flag)) = eye(n_flag);
b(n*PARAMS.n_states+(1:n_flag))=xf(1:n_flag);
MAIN.A =A;


G = [];
h = [];
if upper_boundary_sigma_on
    % upper bound on sigma (linear)
    Gt = zeros(n,n*(PARAMS.n_states+PARAMS.n_controls));
    ht = zeros(n,1);
    for ii = 1:n
        Gt(ii,(ii-1)*(PARAMS.n_states+PARAMS.n_controls)+(PARAMS.n_states:PARAMS.n_states+PARAMS.n_controls)) = [mu2(ii) 0 0 0 1];
        ht(ii) = mu2(ii)*(1+z_u(ii));
    end
    
    G = [G;Gt];
    h = [h;ht];
    dims.l = size(G,1);
end

% no sub-surface constraint
if no_sub_surface_on
    % lower boundary on r_z (linear)
    Gt = zeros(n-1,n*(PARAMS.n_states+PARAMS.n_controls));
    ht = zeros(n-1,1);
    for ii = 1:n-1
        Gt(ii,(ii)*(PARAMS.n_states+PARAMS.n_controls)+3) = -1;
        ht(ii) = 0;
    end
    
    G = [G;Gt];
    h = [h;ht];
    dims.l = size(G,1);
end

if limits_z_on
    % upper boundary on z (linear)
    Gt = zeros(n,n*(PARAMS.n_states+PARAMS.n_controls));
    ht = zeros(n,1);
    for ii = 1:n
        Gt(ii,(ii-1)*(PARAMS.n_states+PARAMS.n_controls)+PARAMS.n_states) = 1;
        ht(ii) = z_u(ii);
    end
    
    G = [G;Gt];
    h = [h;ht];
    dims.l = size(G,1);
    
    % lower boundary on z (linear)
    Gt = zeros(n,n*(PARAMS.n_states+PARAMS.n_controls));
    ht = zeros(n,1);
    for ii = 1:n
        Gt(ii,(ii-1)*(PARAMS.n_states+PARAMS.n_controls)+PARAMS.n_states) = -1;
        ht(ii) = -z_l(ii);
    end
    G = [G;Gt];
    h = [h;ht];
    dims.l = size(G,1);
    
end

dims.q = [];

% put constraint on sigmas: ||u||<sigma
if limits_control_on
    for ii = 1:n
        % init conic-related matrices
        At = zeros(3,n*(PARAMS.n_states+PARAMS.n_controls));
        bt = zeros(3,1);
        ct = zeros(1,n*(PARAMS.n_states+PARAMS.n_controls));
        idx = 1:3;
        jdx = (ii-1)*(PARAMS.n_states+PARAMS.n_controls)+PARAMS.n_states+(1:PARAMS.n_controls-1);
        At(idx,jdx) = eye(3);
        bt(idx,1) = 0;
        ct(1,jdx(end)+1) = 1;
        dt = 0;
        Gt = -[ct;At];
        ht  = [dt;bt];
        
        
        G = [G;Gt];
        h = [h;ht];
        
        dims.q = [dims.q 4];
        
    end
end

% lower boundary on sigma (conic)
if lower_boundary_sigma_on
    Gt = zeros(4,n*(PARAMS.n_states+PARAMS.n_controls));
    ht = zeros(4,1);
    
    for ii = 1:n
        A_mu = zeros(1,n*(PARAMS.n_states+PARAMS.n_controls));
        b_mu = zeros(1,n*(PARAMS.n_states+PARAMS.n_controls));
        c_mu = mu1(ii)*(1+z_l(ii))+.5*mu1(ii)*z_l(ii).^2;
        
        A_mu(:,(ii-1)*(PARAMS.n_states+PARAMS.n_controls)+(PARAMS.n_states:PARAMS.n_states+PARAMS.n_controls)) = [sqrt(2*mu1(ii))/2 0 0 0 0];
        b_mu(1,(ii-1)*(PARAMS.n_states+PARAMS.n_controls)+(PARAMS.n_states:PARAMS.n_states+PARAMS.n_controls)) = -[(mu1(ii)+z_l(ii)*mu1(ii)) 0 0 0 1];
        
        Ac = [b_mu/2; A_mu];
        bc = [c_mu/2+1/2; 0];
        cc = -b_mu/2;
        dc = 1/2-c_mu/2;
        
        Gt = -[cc;Ac];
        ht =  [dc;bc];
        
        G = [G;Gt];
        h = [h;ht];
        
        dims.q = [dims.q 3];
        
    end
end

% glideslope constraint
if glideslope_on
    theta_alt = CONSTANTS.theta_limit;
    
    S = zeros(2,n*(PARAMS.n_states+PARAMS.n_controls));
    s = zeros(1,n*(PARAMS.n_states+PARAMS.n_controls));
    bc = [0;0];
    dc = 0;
    for ii = 2:n
        Ac = S;
        Ac(1,(ii-1)*(PARAMS.n_states+PARAMS.n_controls)+(1:3)) = [tan(theta_alt) 0 0];
        Ac(2,(ii-1)*(PARAMS.n_states+PARAMS.n_controls)+(1:3)) = [0 tan(theta_alt) 0];
        
        cc = s;
        cc(1,(ii-1)*(PARAMS.n_states+PARAMS.n_controls)+(1:3)) = [0 0 1];
        
        Gt = -[cc;Ac];
        ht = [dc;bc];
        
        G = [G;Gt];
        h = [h;ht];
        
        dims.q = [dims.q 3];
        
    end
end

G = sparse(G);
A = sparse(A);

opts = ecosoptimset;

[x, y, info_, s, z] = ecos(C, G, h, dims, A, b, opts);

y =  -y(ns+(1:(n-1)*ns));

if strcmp(info_.infostring,'Optimal solution found')
    status = 1;
else
    status = 2;
end
cpu_time = info_.timing.runtime;


Xc = reshape(x,(PARAMS.n_states+PARAMS.n_controls),[]);
xc = Xc(1:PARAMS.n_states,:);
z = xc(7,:);
xc(7,:) = exp(xc(7,:));
uc = Xc(PARAMS.n_states+1:PARAMS.n_states+PARAMS.n_controls,:);
uc(:,end) = [0;0;uc(4,end);uc(4,end)];
% proper mass is already in


for ii = 1:n
    xdotc(:,ii) = Ac_lti*xc(:,ii)+Bc_lti*uc(:,ii)+Bc_lti*[0;0;-CONSTANTS.g0;0];
end

MAIN.t = tc;
MAIN.x = xc;
MAIN.u = uc;
MAIN.xdot = xdotc;

return
