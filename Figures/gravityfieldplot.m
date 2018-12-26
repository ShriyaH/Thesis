%% change mex function for other asteroids
% codegen -config:mex Poly_gravity -args 
% {zeros(3,1,'double'),zeros(2048,3,'double'),zeros(6138,4,'double'),zeros(4092,3,'double'),zeros(3,3,6138,'double'),zeros(3,3,4092,'double'),zeros(1,1,'double')}

%% Kleopatra grav field
n = Kleopatra.Polyhedron.normalsf;
rho = Kleopatra.rho;
V = Kleopatra.Polyhedron.Vertices;
F = Kleopatra.Polyhedron.Facets;
E = Kleopatra.Polyhedron.Edges;
F_tilde = Kleopatra.Polyhedron.F_tilde;
E_tilde = Kleopatra.Polyhedron.E_tilde;
G = 6.674080000000000e-11; 

tic
x = -20e4:1000:20e4;
[X,Y]=meshgrid(x);
for i = 1:length(X)
    for j = 1:length(X)
        r_A = [X(i,j);Y(i,j);0];
%         [g_A] = Poly_gravity2(r_A,rho,V,F,E,F_tilde,E_tilde,G);
        [g_A] = Poly_gravity_mex(r_A,V,E,F,E_tilde,F_tilde,rho);
        g(i,j) = norm(g_A);
    end
end
toc

figure()
axis equal
patch('Vertices',Kleopatra.Polyhedron.Vertices,'Faces',Kleopatra.Polyhedron.Facets,'FaceVertexCData',[1 1 1],'FaceColor','flat')
hold on
contourf(X,Y,g)
h = colorbar;
ylabel(h, 'Gravitational Accleration (m/s^2)')

