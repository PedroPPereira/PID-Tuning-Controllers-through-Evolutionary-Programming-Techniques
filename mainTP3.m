%% TP3- Decision Systems 2020/21
clear all, close all, clc, warning off
load dados.mat Uv Yv -mat %load data from the real process
load dadosNew.mat Ue Ye -mat %load data from the real process (no transient component)


%% I- Modelacao do Sistema

% Modelo do Sistema pela Equacao Normal
% na = 2; nb = 4; nk = 1; %na = 3; nb = 2; nk = 2;
% arx322 = arx([Ye Ue], [na nb nk]);
% Ai = arx322.A(2:end);
% Bi = arx322.B(nk+1:end);
% theta = [Ai Bi]';
%avaliar modelo
%modelSimulation(theta, Ue, Ye, Uv, Yv); 

% Modelo do Sistema pelo Algoritmo PSO
%restricoes
% UB =  5*ones(1, na+nb); %upper bound
% LB = -5*ones(1, na+nb); %lower bound
%parametros de ajuste
% c1 = 1.49; %SelfAdjustment 
% c2 = 1.49; %SocialAdjustment
%especificacao do particleswarm 
% options = optimoptions(@particleswarm,'MaxIter',300,'SelfAdjustment',c1,...
%     'SocialAdjustment',c2,'SwarmSize',500,'Display','iter');
%handle function
% fun = @(theta)fitness(theta, Ye, Ue); 
%executa algoritmo PSO 
%[theta, fval] = particleswarm(fun, na+nb, LB, UB, options)
%avaliar modelo
%modelSimulation(theta, Ue, Ye, Uv, Yv); 

%optimized theta parameters
theta = [-0.0552   -0.5962    0.0906    0.1255    0.1176    0.0939]';
a1 = theta(1); a2 = theta(2); 
b0 = theta(3); b1 = theta(4); b2 = theta(5); b3 = theta(6);


%% II– Controlo do Sistema

%variables declaration
Ts = 0.08;
Nsamps = 150;
ref = [2*ones(1,Nsamps) 4*ones(1,Nsamps) 3*ones(1,Nsamps) 4.5*ones(1,Nsamps) 3*ones(1,Nsamps)]'; % reference
N = length(ref); 
u = zeros(N,1);
y = zeros(N,1);
t1 = 1:N;
%anti windup PID controller
Kp = 1.6;  % proporcional gain
Ti = .6;   % integral time constant
Td = 0.06; % derivative time constant
Npid = 1;  % ?
K = [Kp Ti Td Npid];
% PID controller w/ GA
%penalties
P = 20; Q = 1; W = 100; %P = 30; Q = 0.5; W = 12;
%handle function
fun = @(K)fitnessPID(K, Ts, theta, P, Q, W, ref);
%restrictions
UB = 5*ones(1, length(K));   %upper bound
LB = [0.1 0.01 0.01 0.01]; %lower bound
%GA specification
options = gaoptimset(@ga);
options = gaoptimset('PopulationType','doubleVector','PopulationSize',250,'CrossoverFraction',...
    0.85,'Generations',40,'SelectionFcn',@selectionroulette,'PlotFcns',@gaplotbestf);
%run GA
[K,fval] = ga(fun,length(K),[],[],[],[],LB,UB,[],[],options);
%plot results
modelSimulationPID(K, Ts, theta, ref); 

fprintf('Kp=%.4f; Ti=%.4f; Td=%.4f\n', K(1), K(2), K(3));




