function [Imp_Ellipse] = Get_Imp_Ell(P, tolerance,Traj)
% [A , c] = MinVolEllipse(P, tolerance)
% Finds the minimum volume enclsing ellipsoid (MVEE) of a set of data
% points stored in matrix P. The following optimization problem is solved: 
%
% minimize       log(det(A))
% subject to     (P_i - c)' * A * (P_i - c) <= 1
%                
% in variables A and c, where P_i is the i-th column of the matrix P. 
% The solver is based on Khachiyan Algorithm, and the final solution 
% is different from the optimal value by the pre-specified amount of 'tolerance'.
%
% inputs:
%---------
% P : (d x N) dimensional matrix containing N points in R^d.
% tolerance : error in the solution with respect to the optimal value.
%
% outputs:
%---------
% A : (d x d) matrix of the ellipse equation in the 'center form': 
% (x-c)' * A * (x-c) = 1 
% c : 'd' dimensional vector as the center of the ellipse. 
% 
% example:
% --------
%      P = rand(5,100);
%      [A, c] = MinVolEllipse(P, .01)
%
%      To reduce the computation time, work with the boundary points only:
%      
%      K = convhulln(P');  
%      K = unique(K(:));  
%      Q = P(:,K);
%      [A, c] = MinVolEllipse(Q, .01)
%
%
% Nima Moshtagh (nima@seas.upenn.edu)
% University of Pennsylvania
%
% December 2005
% UPDATE: Jan 2009



%%%%%%%%%%%%%%%%%%%%% Solving the Dual problem%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% ---------------------------------
% data points 
% -----------------------------------
[d N] = size(P);

Q = zeros(d+1,N);
Q(1:d,:) = P(1:d,1:N);
Q(d+1,:) = ones(1,N);


% initializations
% -----------------------------------
count = 1;
err = 1;
u = (1/N) * ones(N,1);          % 1st iteration


% Khachiyan Algorithm
% -----------------------------------
while err > tolerance
    X = Q * diag(u) * Q';       % X = \sum_i ( u_i * q_i * q_i')  is a (d+1)x(d+1) matrix
    M = diag(Q' * inv(X) * Q);  % M the diagonal vector of an NxN matrix
    [maximum j] = max(M);
    step_size = (maximum - d -1)/((d+1)*(maximum-1));
    new_u = (1 - step_size)*u ;
    new_u(j) = new_u(j) + step_size;
    count = count + 1;
    err = norm(new_u - u);
    u = new_u;
end



%%%%%%%%%%%%%%%%%%% Computing the Ellipse parameters%%%%%%%%%%%%%%%%%%%%%%
% Finds the ellipse equation in the 'center form': 
% (x-c)' * A * (x-c) = 1
% It computes a dxd matrix 'A' and a d dimensional vector 'c' as the center
% of the ellipse. 

U = diag(u);

% the A matrix for the ellipse
% --------------------------------------------
A = (1/d) * inv(P * U * P' - (P * u)*(P*u)' );


% center of the ellipse 
% --------------------------------------------
C = P * u;

% plot the ellipse 
% --------------------------------------------
NN = 20;  % Default value for grid
[L D M] = svd(A);

a = 1/sqrt(D(1,1));
b = 1/sqrt(D(2,2));
c = 1/sqrt(D(3,3));
[X,Y,Z] = ellipsoid(0,0,0,a,b,c,NN);

Imp_Ellipse.X = X;
Imp_Ellipse.Y = Y;
Imp_Ellipse.Z = Z;

XX = zeros(NN+1,NN+1);
YY = zeros(NN+1,NN+1);
ZZ = zeros(NN+1,NN+1);
    for k = 1:length(X)
        for j = 1:length(X)
            point = [X(k,j) Y(k,j) Z(k,j)]';
            P = M * point;
            XX(k,j) = P(1)+C(1);
            YY(k,j) = P(2)+C(2);
            ZZ(k,j) = P(3)+C(3);
            semi = [a;b; c];
            Q = M * semi;
        end
    end

    Imp_Ellipse.A = A;
    Imp_Ellipse.C = C;
    Imp_Ellipse.XX = XX;
    Imp_Ellipse.YY = YY;
    Imp_Ellipse.ZZ = ZZ;
    Imp_Ellipse.sem = Q;
end