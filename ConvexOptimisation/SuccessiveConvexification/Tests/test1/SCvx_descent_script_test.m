%%----Ast_descent_script----%%
clear all; clc; close all; 

global ITR PARAMS CONSTANTS;
ini_models_test;

[xc,uc,xdotc,cpu_time,status] = SCvx_transcription_test();

m_spent = ITR.x_k{end}(1,1)-ITR.x_k{end}(1,end);
disp(['mass spent: ',num2str(m_spent),' kg'])

