function [ index_v,index_f ] = Find_index(Asteroid_file)
%% Function that finds indices of vertices and facets in polyhedron file
%
%%
A=importdata(Asteroid_file);
i=1;
s='v';
while s=='v'
    s=A.rowheaders{i};
    if s=='v'
        index_v=i;
    end
    i=i+1;
end
index_f=size(A.data,1);


end

