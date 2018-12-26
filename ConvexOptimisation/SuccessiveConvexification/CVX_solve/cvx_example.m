Initialize_models_test;
[A,b,G,h,C,dims] = SCvx_ini_para_test();
n = 810;
cvx_begin
    variable x(n)
    minimize( C' * x  )
    subject to
        A * x == b;
         norm( G*x, 2 ) <= h;
cvx_end