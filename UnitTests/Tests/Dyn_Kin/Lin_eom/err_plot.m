j=1:41;
y_n(j,1) = y(j,1);
y_n(j,2) = norm(y(j,2:4));
y_n(j,3) = norm(y(j,5:7));
y_n(j,4:7) = y(j,8:11);
y_n(j,8) = norm(y(j,12:14));
y_n(j,9) = norm(y(j,15:17));
y_n(j,10) = norm(y(j,18:20));
j=1:41;
y_n1(j,1) = y1(j,1);
y_n1(j,2) = norm(y1(j,2:4));
y_n1(j,3) = norm(y1(j,5:7));
y_n1(j,4:7) = y1(j,8:11);
y_n1(j,8) = norm(y1(j,12:14));
y_n1(j,9) = norm(y1(j,15:17));
y_n1(j,10) = norm(y1(j,18:20));

err = abs(y_n-y_n1);

figure()
hold on
plot(t,err(j,1));
plot(t,err(j,2));
plot(t,err(j,3));
plot(t,err(j,8));
plot(t,err(j,9));
plot(t,err(j,10));

figure()
hold on
plot(t,y(:,2))
plot(t,y(:,3))
plot(t,y(:,4))
plot(tspan,y1(:,2))
plot(tspan,y1(:,3))
plot(tspan,y1(:,4))
xlabel('Time (s)');
ylabel('Position Components');
legend('r_1','r_2','r_3','Location','northeastoutside');
title('r^{I} (U_l)');
grid on
print -depsc err_p
    