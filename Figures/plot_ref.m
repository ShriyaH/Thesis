function plot_ref(o,a,color)
% o: origin, a,b,c : axes vectors

    if size(a)== [3 3]
        hold on
        quiver3(o(1,1),o(1,2),o(1,3),a(1,1),a(1,2),a(1,3),color);
        quiver3(o(1,1),o(1,2),o(1,3),a(2,1),a(2,2),a(2,3),color);
        quiver3(o(1,1),o(1,2),o(1,3),a(3,1),a(3,2),a(3,3),color);
    elseif size(a) == [1 3]
        quiver3(o(1,1),o(1,2),o(1,3),a(1,1),a(1,2),a(1,3),color);
    end
end