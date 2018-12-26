clear all
close all
% state=zeros(100);
% LT=zeros(1);

cspice_furnsh( 'SPICE/standard_R.txt' )


epoch1    = 'July 18, 2016 12:00 PM UTC';
epoch2    = 'October 25, 2014 12:00 PM UTC';

et1 = cspice_str2et( epoch1 );
et2 = cspice_str2et( epoch2 );

ett=et1:0.05:(et1+4e4);
    
% [a,b,c,d]=cspice_frinfo ( 915500); 
[state, lt] = cspice_spkezr('KLEOPATRA', et1,'J2000', 'NONE', 'SUN');


% radii = cspice_bodvrd( 'ITOKAWA', 'RADII', 3 )
% C = cspice_pxform('ITOKAWA_FIXED' , 'J2000', ett );

cspice_unload( 'SPICE/standard_R.txt' )

R_I=state(1:3,:);
V_I=state(4:6,:);
% for i=1:1:size(R_A,2)
% R_I(:,i)=C(:,:,i)*R_A(:,i);
% end
% Time=ett-et1;

%% Plotting
% 
% dvx=diff(V_I(1,:));
% dvy=diff(V_I(2,:));
% dvz=diff(V_I(3,:));
% 
% % for i=1:1:size(R_I,2)
% %     phi(i)=acosd(dot(R_I(:,i)/norm(,[1 0 0]));
% % end
% figure 
% plot(Time(1:(end-1)), dvx,Time(1:(end-1)), dvy,Time(1:(end-1)), dvz);
% % 
% fsz=11;
% ppsz=[8 8]; 
% mrg=0.1;
% % 
% goodplot(ppsz,mrg,fsz)
