function [lm_coord,lm_n,lm_r] = Get_landmarks(Asteroid,n)
% n = number of wanted landmarks

v = Asteroid.Polyhedron.Vertices;
f = Asteroid.Polyhedron.Facets;
a = Asteroid.Polyhedron.A_facet';
nf = Asteroid.Polyhedron.normalsf/norm(Asteroid.Polyhedron.normalsf);

a_norm = a/sum(a);  %norm of facet areas to put more weight on larger areas
[N,~,index] = histcounts(rand(n,1),cumsum([0,a_norm]));  %get index of facets with number of landmarks(N) in them

r1 = rand(n,1); 
r2 = sqrt(rand(n,1));

lm_coord = v(f(index,1),:).*r1.*r2 + v(f(index,2),:).*(1-r1).*r2 + v(f(index,3),:).*(1-r2); %generate random landmarks at the indexed facets 
lm_n = nf(index,:); %normals at the landmarks

r=[];
for i= 1:length(N)
    if N(1,i) ~= 0
        r(1,i) = sqrt(a(1,i)/N(1,i));  %radius of the landmarks dependent on area of the facet
    elseif N(1,i) == 0
        r(1,i) = 0;
    end
end  
lm_r = r(1,index)*rand*0.2;

end

    

