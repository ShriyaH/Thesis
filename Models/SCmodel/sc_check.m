
SC.dim.block = [2.1 2 2.8]; %SC dimensions, m
SC.dim.SA = [0 14 2.28]; %Solar array dimensions, m

x = SC.dim.block(1)/2;
y = SC.dim.block(2)/2;
z = SC.dim.block(3)/2;

sy = SC.dim.SA(2)/5;
sz = SC.dim.SA(3)/2;
d = 0.06;
d1 = 1+y;

block = [x -y -z
      x y -z
     -x y -z
     -x -y -z
     x -y z
      x y z
      -x y z
     -x -y z];
 
SA1 = [0 d1 -sz 
      0 d1 sz 
      0 d1+sy -sz  
      0 d1+sy sz 
      0 d1+sy+d -sz  
      0 d1+sy+d sz  
      0 d1+2*sy+d -sz 
      0 d1+2*sy+d sz 
      0 d1+2*sy+2*d -sz 
      0 d1+2*sy+2*d sz 
      0 d1+3*sy+2*d -sz 
      0 d1+3*sy+2*d sz 
      0 d1+3*sy+3*d -sz 
      0 d1+3*sy+3*d sz 
      0 d1+4*sy+3*d -sz 
      0 d1+4*sy+3*d sz 
      0 d1+4*sy+4*d -sz 
      0 d1+4*sy+4*d sz 
      0 d1+5*sy+4*d -sz 
      0 d1+5*sy+4*d sz]; 
SA2 = -SA1;
Polyhedron.Vertices = [block;SA1;SA2];
       
Polyhedron.Facets = [1 2 6
    1 6 5
    2 3 7
    2 7 6
    3 4 8
    3 8 7
    4 1 5
    4 5 8
    4 3 2
    4 2 1
    5 6 7
    5 7 8
    9 11 12
    9 12 10
    13 15 16
    13 16 14
    17 19 20
    17 20 18
    21 23 24
    21 24 22
    25 27 28
    25 28 26
    29 32 30
    29 31 32
    33 36 34
    33 35 36
    37 40 38
    37 39 40
    41 44 42
    41 43 44
    45 48 46
    45 47 48];

% [Polyhedron.Facets, Polyhedron.Vertices] = stlread('Rosetta.stl');
% [nf,nv] = reducepatch(SC.Polyhedron.Facets,SC.Polyhedron.Vertices,0.02);
[Polyhedron.normalsf,Polyhedron.R_1,Polyhedron.R_2,Polyhedron.R_3,Polyhedron.C,Polyhedron.A_facet] = SC_prop(Polyhedron);

SC.Polyhedron = Polyhedron;

patch('Vertices',SC.Polyhedron.Vertices,'Faces',SC.Polyhedron.Facets,'FaceVertexCData',[1 1 1],'FaceColor','flat');
axis equal



