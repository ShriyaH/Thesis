l = [4 5 6];
l = l./norm(l);
t = [1 2 3];

a = cross(l,cross(t,l,2),2);
d = (t - a)./l;
d1 = dot(l,t);

