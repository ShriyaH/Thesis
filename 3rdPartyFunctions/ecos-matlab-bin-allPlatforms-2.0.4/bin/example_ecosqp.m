% set dimensions and sparsity of A
n = 1000; m = 10; density = 0.01; 

% linear term
c = [zeros(n,1); ones(n,1)];

% equality constraints
A = sprandn(m,n,density);
Atilde = [A, zeros(m,n)];
b = randn(m,1);

% linear inequality constraints
I = speye(n);
G = [  I -I;
      -I -I];
h = zeros(2*n,1);

% cone dimensions (LP cone only)
dims.l = 2*n; dims.q = [];

% call solver
z = ecos(c,G,h,dims,Atilde,b);

lb = -1000*ones(size(z,1),1);
ub =  1000*ones(size(z,1),1);
H = eye(size(z,1),size(z,1));
f = c;
[z,fval,exitflag,output,lambda,t] = ecosqp(H,f,G,h,Atilde,b,lb,ub)

x = z(1:n);
nnzx = sum(abs(x) > 1e-8);

% print sparsity info
fprintf('Optimal x has %d/%d (%4.2f%%) non-zero entries.\n', nnzx , n,  nnzx/n*100);
