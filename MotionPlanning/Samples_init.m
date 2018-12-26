function [Samples,n_c,nf,nv] = Samples_init(n_v,r,dv_max,frac,Asteroid)
dv_max_mag = norm(dv_max);
rng(0,'twister');        %Comment out if different regenration of initial samples needed 

%% Generate Configuration Space samples

n = Asteroid.Polyhedron.normalsf;
V = Asteroid.Polyhedron.Vertices;
f = Asteroid.Polyhedron.Facets;
h = r.*ones(size(V)).*sign(V);
vert = V+h;
if frac > 0
    vv = patch('Vertices',vert,'Faces',Asteroid.Polyhedron.Facets,'FaceVertexCData',[0 1 0],'FaceAlpha',0.3, 'EdgeColor', [1 0 0]);
%     vv = patch('Vertices',vert,'Faces',Asteroid.Polyhedron.Facets,'FaceVertexCData',[0 1 0],'FaceAlpha',0.3, 'EdgeColor', [1 0 0], 'visible','off');
    [nf,nv] = reducepatch(vv,frac);
elseif frac == 0
    nf = f;
    nv = vert;    
end
n_c = length(nv);

%% Generate Delta V samples
el = asin(2*rand(n_v,1)-1); %elevation of samples between pi/2 and -pi/2
az = 2*pi*rand(n_v,1);      %azimuth of samples between 0 and 2pi
radii = dv_max_mag*(rand(n_v,1).^(1/3));  %max radii of sphere for samples
[u, v, w] = sph2cart(az,el,radii);
dv = [u v w];

Samples.mag_v = sqrt(u.^2+v.^2+w.^2);
% Scores.dv = mag_v./sum(mag_v);

%% Generate Delta t 
% dt = tf*rand(n_v,1);

%% Velocity-Space Control Domain
% Samples.U_sph_init = [dt dv];

%% Reachable Domain

for i = 1:length(dv)
    Samples.D_sph_init(1:3,:,i) = vert(:,1:3)';
    for j = 1:length(vert)
    Samples.D_sph_init(4:6,j,i) = dv(i,1:3)';
    end
end
end



