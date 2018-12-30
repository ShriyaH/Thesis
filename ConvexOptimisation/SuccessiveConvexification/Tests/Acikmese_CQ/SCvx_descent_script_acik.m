%%----Ast_descent_script----%%
% Asteroid descent problem for ECOS with Successive Convexification
% Shriya Hazra, 31-Jul-2018 
clear all; 
clc; 
close all; 

global ITR PARAMS CONSTANTS;
ini_models_acik;

[xc,uc,xdotc,cpu_time,status] = SCvx_transcription_acik();

m_spent = ITR.x_k{end}(1,1)-ITR.x_k{end}(1,end);
disp(['mass spent: ',num2str(m_spent),' kg'])

