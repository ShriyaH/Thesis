function h = circle(x,y,z,r,t)
hold on

th = 0:pi/50:2*pi;
for i = 1:length(th)
    if i <= length(th)/2
    xunit(i) = r * cos(t) * cos(th(i)) + x;
    yunit(i) = r * cos(t) * sin(th(i)) + y;
    zunit(i) = r * sin(t) + z;
    else
    xunit(i) = r * cos(-t) * cos(th(i)) + x;
    yunit(i) = r * cos(-t) * sin(th(i)) + y;
    zunit(i) = r * sin(-t) + z;
    end
end

% r_AI = [xunit;yunit;zunit];
% for i = 1:length(r_AI)
% r_BI(:,i) = quat_trans(q_BA,r_AI(:,i),'vect');
% end
h = plot3(xunit, yunit, zunit);
hold off
end