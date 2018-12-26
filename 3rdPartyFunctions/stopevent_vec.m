function [value, isterminal, direction] = stopevent_vec(T,Y)
G = 6.67408e-11;

[Cube] = Get_Cube(60e3,30e3,40e3,G);        %Get the asteroid polyhedron and properties

[Imp_Ellipse] = MinVolEllipse(Cube.Polyhedron.Vertices', 0.001); %Minimum volume impact ellipsoid 

a = Imp_Ellipse.ell_ratio(1); %axes of minimum ellipsoid
b = Imp_Ellipse.ell_ratio(2);
c = Imp_Ellipse.ell_ratio(3);
mu = Cube.mu;

Y =Y';
r = sqrt(Y(1,:).^2 + Y(2,:).^2 + Y(3,:).^2);
imp = (Y(1,:)/a).^2 + (Y(2,:)/b).^2 + (Y(3,:)/c).^2;                     %Instantaneous ellipsoid
esc = sqrt((mu*Y(1,:)./r.^3).^2 + (mu*Y(2,:)./r.^3).^2 + (mu*Y(3,:)./r.^3).^2);  %Instantaneous acceleration
esc2 = sqrt(3*mu./esc)/4;                                          %Maximum allowable acceleration at that position

% value = [sqrt(Y(1)^2/+Y(2)^2+Y(3)^2)-7.8e04; sqrt(Y(1)^2+Y(2)^2+Y(3)^2)-2.5e05]; 
% value = [imp-1; esc2-esc];    %Impact condition, Escape condition
% isterminal = [1; 1];          %Stop the integration
% direction  = [0; 0];          %Bi-directional approach
value = imp-1    %Impact condition, Escape condition
isterminal = 1;          %Stop the integration
direction  = -1;          %Bi-directional approach
end
