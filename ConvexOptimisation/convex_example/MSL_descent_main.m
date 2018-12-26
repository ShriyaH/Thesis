function[] = MSL_descent_main
% this function defines all the parameters, the constraints and the
% functions defining the mars descent and landing optimal problem 

global CONSTANTS;


% constants of the problem
CONSTANTS.g0 = 3.7114;
CONSTANTS.sigma = 1/(225*9.807*cos(27*pi/180)); 
CONSTANTS.T_max = 3100; 
CONSTANTS.m0 = 1905;
CONSTANTS.mf = 1505;

CONSTANTS.T = [0.3; 0.8]*CONSTANTS.T_max;
CONSTANTS.soft_on = 0;
CONSTANTS.x0 = [1500; 2000; -75; 100; CONSTANTS.m0];
CONSTANTS.xf = [0;0;0.1;0.1;CONSTANTS.mf];
CONSTANTS.xf = [0;0;0;0;CONSTANTS.mf];
CONSTANTS.tf = 81;
