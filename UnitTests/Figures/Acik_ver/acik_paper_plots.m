
%%plot 6dof acik
%converged mass
x = [0,1,2,3,4,5,6,7,8,9];
y = [0.00018947,0.00017263,0.00016421,0.00014737,0.0001421,0.00013368,...
    0.00008421,0.00008947,0.00008421,0.00007895];
figure()
plot(x,y,'Linewidth',2,'Marker','.', 'Color', 'r')
grid on

%dynamic relaxation
x = [1,2,3,4,5,6,7,8,9,10];
y = [0.93194,0.87958,0.87958,0.86387,0.8377,0.68063,0.43979,0.44503,0.43979,0.44503];
figure()
plot(x,y,'Linewidth',2,'Marker','.', 'Color', 'r')
grid on

%trust region
x = [1,2,3,4,5,6,7,8,9,10];
y = [6.23e10,6.18e10,5.97e10,5.86e10,5.92e10,5.65e10,3.40e10,2.41e10,2.46e10,2.46e10];
figure()
plot(x,y,'Linewidth',2,'Marker','.', 'Color', 'r')
grid on

%angular rate
x = [0,0.172413793103448,0.344827586206897,0.517241379310345,0.689655172413793,...
    0.862068965517241,1.03448275862069,1.20689655172414,1.37931034482759,1.55172413793103,...
    1.72413793103448,1.89655172413793,2.06896551724138,2.24137931034483,2.41379310344828,...
    2.58620689655172,2.75862068965517,2.93103448275862,3.10344827586207,3.27586206896552,...
    3.44827586206897,3.62068965517241,3.79310344827586,3.96551724137931,4.13793103448276,...
    4.31034482758621,4.48275862068966,4.65517241379310,4.82758620689655,5];
y = [-3.35e-08,-6.053,-16.316,-26.053,-29.737,-22.895,-12.105,-3.421,1.316,...
    6.053,13.947,20.789,24.737,26.842,26.842,25,23.158,21.579,18.947,13.684,5,...
    -5,-13.947,-19.737,-21.053,-19.737,-16.579,-10.789,-3.947,-2.12e-08];
figure()
plot(x,y,'Linewidth',2,'Marker','.', 'Color', 'r')
grid on

%gimbal angle
x = [0,0.172413793103448,0.344827586206897,0.517241379310345,0.689655172413793,...
    0.862068965517241,1.03448275862069,1.20689655172414,1.37931034482759,1.55172413793103,...
    1.72413793103448,1.89655172413793,2.06896551724138,2.24137931034483,2.41379310344828,...
    2.58620689655172,2.75862068965517,2.93103448275862,3.10344827586207,3.27586206896552,...
    3.44827586206897,3.62068965517241,3.79310344827586,3.96551724137931,4.13793103448276,...
    4.31034482758621,4.48275862068966,4.65517241379310,4.82758620689655,5];
y = [0.1485,-9.9505,-9.9505,-7.9703,1.8317,10.0495,10.2475,6.4851,3.7129,...
    6.3861,10.2475,10.2475,10.1485,6.2871,-5.8911,-9.9505,-9.8515,-9.8515,...
    -9.9505,-9.9505,-9.9505,-9.9505,-9.8515,-8.1683,2.6238,10.2475,10.1485,10.1485,10.1485,0.0495];
figure()
plot(x,y,'Linewidth',2,'Marker','.', 'Color', 'r')
grid on

%thrust
x = [0,0.172413793103448,0.344827586206897,0.517241379310345,0.689655172413793,...
    0.862068965517241,1.03448275862069,1.20689655172414,1.37931034482759,1.55172413793103,...
    1.72413793103448,1.89655172413793,2.06896551724138,2.24137931034483,2.41379310344828,...
    2.58620689655172,2.75862068965517,2.93103448275862,3.10344827586207,3.27586206896552,...
    3.44827586206897,3.62068965517241,3.79310344827586,3.96551724137931,4.13793103448276,...
    4.31034482758621,4.48275862068966,4.65517241379310,4.82758620689655,5];
y = [1.9934,2.8061,3.0155,2.9888,3.0146,3.0141,3.0006,3.0001,2.9997,2.8943,...
    2.4086,1.5557,0.8208,0.4925,0.492,0.5571,0.5174,0.5431,1.0804,2.0504,2.8238,...
    2.902,2.272,1.1962,0.54,0.6313,1.2735,1.9157,1.8365,1.0623];
figure()
plot(x,y,'Linewidth',2,'Marker','.', 'Color', 'r')
grid on

%tilt angle
x = [0,0.172413793103448,0.344827586206897,0.517241379310345,0.689655172413793,...
    0.862068965517241,1.03448275862069,1.20689655172414,1.37931034482759,1.55172413793103,...
    1.72413793103448,1.89655172413793,2.06896551724138,2.24137931034483,2.41379310344828,...
    2.58620689655172,2.75862068965517,2.93103448275862,3.10344827586207,3.27586206896552,...
    3.44827586206897,3.62068965517241,3.79310344827586,3.96551724137931,4.13793103448276,...
    4.31034482758621,4.48275862068966,4.65517241379310,4.82758620689655,5];
y = [-0.0002,0.401,2.185,5.942,10.685,15.626,18.593,19.785,20.186,19.601,...
    18.029,14.878,11.134,6.404,1.675,-2.661,-6.799,-10.739,-14.088,-17.239,...
    -18.614,-18.804,-17.021,-14.251,-10.297,-6.738,-3.968,-1.197,-0.007,0.395];
figure()
plot(x,y,'Linewidth',2,'Marker','.', 'Color', 'r')
grid on