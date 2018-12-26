N = 30;
K = N;

ns = 20;
nc = 3;
nv = 3;

At = zeros(ns,ns);
Bt = zeros(ns,nc);
Vt = zeros(ns,nv);
zt = zeros(ns,1);

Ad = ones(ns,ns);
Bd = 2.*ones(ns,nc);
Vd = 3.*ones(ns,nv);
zd = 4.*ones(ns,1);


A = zeros((K+1)*ns,N*(ns+nc+nv));
A(1:ns,1:ns) = eye(ns);

for ii = 1:K
    jj = ii;
    idx = ii*ns+(1:ns);
    
    %state discrete matrices
    jdx = (jj-1)*(ns+nc+nv)+(1:ns);
    A(idx,jdx) = -Ad;
    
    %control discrete matrices
    jdx = ns+(jj-1)*(ns+nc+nv)+(1:nc);
    A(idx,jdx) = -Bd;
    
    %virtual control discrete matrices
    jdx = ns+nc+(jj-1)*(ns+nc+nv)+(1:nv);
    A(idx,jdx) = -Vd;
    
    %next step state
    jdx = ns+nc+nv+(jj-1)*(ns+nc+nv)+(1:ns);
    A(idx,jdx) = eye(ns);
end