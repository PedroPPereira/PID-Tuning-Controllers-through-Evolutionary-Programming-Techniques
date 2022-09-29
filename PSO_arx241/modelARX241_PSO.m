%% TP3- Decision Systems 2020/21
clear all, close all, clc, warning off
load dados.mat Uv Yv -mat %load data from the real process
load dadosNew.mat Ue Ye -mat %load data from the real process (no transient component)

%main variables declaration
Nsamps = 150;
ref = [2*ones(1,Nsamps) 4*ones(1,Nsamps) 3*ones(1,Nsamps) 4.5*ones(1,Nsamps) 3*ones(1,Nsamps)]'; % reference

%b) Modelo do Sistema pela Equacao Normal
na = 2; nb = 4; nk = 1;
arx241 = arx([Ye Ue], [na nb nk]);
Ai = arx241.A(2:end);
Bi = arx241.B(nk+1:end);
theta = [Ai Bi]';
%avaliar modelo
modelSimulation241(theta, Ue, Ye, Uv, Yv);

%b) Modelo do Sistema pelo Algoritmo PSO
%restricoes
UB =  5*ones(1, na+nb); %upper bound
LB = -5*ones(1, na+nb); %lower bound
%parametros de ajuste
c1 = 1.49; %SelfAdjustment 
c2 = 1.49; %SocialAdjustment
%especificacao do particleswarm 
options = optimoptions(@particleswarm,'MaxIter',300,'SelfAdjustment',c1,...
    'SocialAdjustment',c2,'SwarmSize',500,'Display','iter');
%handle function
fun = @(theta)fitness241(theta, Ye, Ue);
%executa algoritmo PSO 
[theta, fval] = particleswarm(fun, na+nb, LB, UB, options)
%avaliar modelo
modelSimulation241(theta, Ue, Ye, Uv, Yv);
theta
theta = [-0.0552   -0.5962    0.0906    0.1255    0.1176    0.0939]';
a1 = theta(1); a2 = theta(2); 
b0 = theta(3); b1 = theta(4); b2 = theta(5); b3 = theta(6);





