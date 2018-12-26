function [  ] = Ref_frame_plot(DCM, R)
%% Function to plot a refererce frame given its attitude (DCM), position,
%  and color

i=DCM(1,:);
j=DCM(2,:);
k=DCM(3,:);
scale=1;
% quiver3(R(1), R(2), R(3), i(1),i(2),i(3),scale,'Color','red', 'Linewidth',2);
% text(i(1)+R(1),i(2)+R(2), i(3)+R(3), 'X')
% hold on
% quiver3(R(1), R(2), R(3),j(1),j(2),j(3),scale,'Color','green','Linewidth',2);
% text(j(1)+R(1),j(2)+R(2), j(3)+R(3), 'Y')
% hold on
% quiver3(R(1), R(2), R(3),k(1),k(2),k(3),scale,'Color','blue','Linewidth',2);
% text(k(1)+R(1),k(2)+R(2), k(3)+R(3), 'Z')

hold on
axis equal
end

