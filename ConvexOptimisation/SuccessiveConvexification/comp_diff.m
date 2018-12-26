%% plot non-linear function
% F = @(x,y,z) (cos(x)./((sin(y)).^3+z)); @(q1,q2,q3,q4,q5,q6,q7,q8)
r = [2,3,5]';
%  sym q1 q2 q3 q4 q5 q6 q7 q8 h
F =@(q1,q2,q3,q4,q5,q6,q7,q8) log(norm(r-2*[-q8*q1 + q7*q2 - q6*q3 + q5*q4; -q7*q1 - q8*q2 + q5*q3 + q6*q4; q6*q1 - q5*q2 - q8*q3 + q7*q4])/q2);

% F = @(q1,q2,q3,q4,q5,q6,q7,q8) r-f;

% fplot3(F,F,F,[-pi/4,pi/2])
% axis([-pi/4,pi/2,0,6],[-pi/4,pi/2,0,6],[-pi/4,pi/2,0,6])
% set(gca,'xtick',[-pi/4,0,pi/4,pi/2],'ytick',[-pi/4,0,pi/4,pi/2],'ztick',[-pi/4,0,pi/4,pi/2])
% line([pi/4,pi/4],[F(pi/4,pi/4),F(pi/4,pi/4)],'marker','.','markersize',18)

%% finite and complex differentiation comparison
% Fpc = @(x,y,z,h) imag(F(x+i*h,y+i*h,z+i*h))./h;
% Fpd = @(x,y,z,h) (F(x+h,y+h,z+h) - F(x-h,y-h,z-h))./(2*h);
Fpc =@(q1,q2,q3,q4,q5,q6,q7,q8,h) imag(F(q1+i*h,q2+i*h,q3+i*h,q4+i*h,q5+i*h,q6+i*h,q7+i*h,q8+i*h))./h;
Fpd =@(q1,q2,q3,q4,q5,q6,q7,q8,h) (F(q1+h,q2+h,q3+h,q4+h,q5+h,q6+h,q7+h,q8+h) - F(q1-h,q2-h,q3-h,q4-h,q5-h,q6-h,q7-h,q8-h))./(2*h);

% format long
%    disp(['           h           a   complex step   b  c  finite differerence e'])
%    for h = 10.^(-(1:16))
%       disp([h Fpc(pi/4,pi/4,pi/4,h) Fpd(pi/4,pi/4,pi/4,h)])
%    end

format long
   disp(['           h           complex step   finite differerence '])
   for h = 10.^(-(1:16))
      disp([h Fpc(0.182574185835055,0.365148371670111,0.547722557505166,0.730296743340221,10e3,20e3,50e3,0,h) Fpd(0.182574185835055,0.365148371670111,0.547722557505166,0.730296743340221,10e3,20e3,50e3,0,h)])
   end
