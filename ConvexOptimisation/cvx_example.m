cvx_setup
m = 20; n = 10; p = 4;

A = randn(m,n); b = randn(m,1);
C = randn(p,n); d = randn(p,1); e = rand;

cvx_begin

cvx_solver ecos  % Set ECOS as the solver
variable x(n)

minimize( norm( A * x - b, 2 ) )
subject to
    C * x == d
    norm( x, Inf ) <= e

cvx_end 