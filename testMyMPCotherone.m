clear variables
close all
clc
%% SimscapeCrane_MPC_start;
load('Params_Simscape.mat');
load('SSmodelParams.mat');
%% Load the dynamics matrices using a solution from last assignment
Ts=1/20; % inc. improves non-linear performance, also increases computation time
% dec just shit
[A,B,C,~] = genCraneODE(m,M,MR,r,g,Tx,Ty,Vm,Ts);

%% Define other parameters
N=25; % ceiling to ensure N is an integer
    % inc improves slightly, inc comp time
T=20;

% constant to correct for "stickiness" of the crane in X axis
stickCorr = 0;

% define sides of the square
xHigh = 0.4 + stickCorr;
xLow = 0.2 - stickCorr;
yHigh = 0.5;
yLow = 0.3;

% define target states
xTarget1=[xHigh 0 yHigh 0 0 0 0 0]';
xTarget2=[xLow 0 yHigh 0 0 0 0 0]';
xTarget3=[xLow 0 yLow 0 0 0 0 0]';
xTarget4=[xHigh 0 yLow 0 0 0 0 0]';
xTarget = [xTarget1, xTarget2, xTarget3, xTarget4];

x0=[xHigh 0 yHigh 0 0 0 0 0]'; % starting offset

xZero = xHigh;
yZero = yHigh;

%% Declare penalty matrices and tune them here:
Q=eye(8); % increasing improves linear performance, non-linear moves in Y but barely in X
              % making it smaller => shit don't move at all
Q(1,1) = 200;       % increase this hard to make X move
% Q(2,2) = 1;
Q(3,3) = 200;
% Q(4,4) = 1;
Q(5,5) = 200;
% Q(6,6) = 10;
Q(7,7) = 200;
% Q(8,8) = 10;

R=eye(2); % increase and shit don't move
              % decrease non-linear doesn't move
P=eye(8);     % increase and it barely moves
            % decrease and it's even worse
P(1,1) = 500;       % increase this hard to make X move

P(3,3) = 500;

%% Declare contraints
% Declaring constraints only on states (X,Y,theta,psi) and inputs u
angleConstraint=1.5*pi/180; % in radians
cl=[xLow; yLow; -angleConstraint; -angleConstraint];
ch=[xHigh; yHigh;  angleConstraint;  angleConstraint];
ul=[-1; -1];
uh=[1; 1];
% constrained vector is Dx, hence
D=zeros(4,8);D(1,1)=1;D(2,3)=1;D(3,5)=1;D(4,7)=1;

%% Compute stage constraint matrices and vector
[Dt,Et,bt]=myStageConstraints(A,B,D,cl,ch,ul,uh);

%% Compute trajectory constraints matrices and vector
[DD,EE,bb]=myTrajectoryConstraints(Dt,Et,bt,N);

%% Compute QP constraint matrices
[Gamma,Phi] = genPrediction(A,B,N); % get prediction matrices:
[F,J,L]=myConstraintMatrices(DD,EE,Gamma,Phi,N);


%% Compute QP cost matrices
[H,G] = genCostMatrices(Gamma,Phi,Q,R,P,N);

%% Prepare cost and constraint matrices for mpcqpsolver
% Calculating the inverse of the lower triangular H. see doc mpcqpsolver.
[H,p] = chol(H,'lower');
H=(H'\eye(size(H)))';

%% Running a matlab simulation and visualisng the results:
MatlabSimulation
GantryResponsePlot(t,allU,...
    x,[-1 -1],[1 1],[0 0],[xRange(2) yRange(2)],[1 3],xTarget,'Linear simulation: MPC performance');

%% plot trace of the load to check square tracking
stringLength = 1;
figure;
scatter(x(:,1)+stringLength*sin(x(:,5)),x(:,3)-stringLength*sin(x(:,7)))
hold on
scatter(x(:,1),x(:,3),'g');
scatter(xLow,yLow, 'r');
scatter(xLow,yHigh, 'r');
scatter(xHigh,yHigh, 'r');
scatter(xHigh,yLow, 'r');

%% Run the Simulink simulation for your controller
% Note that in order to test your controller you have to navigate to
% SimscapeCrane_MPChard/MPC and copy paste your controller code inside the
% Matlab Function block

SimscapeCrane_MPChard
sim('SimscapeCrane_MPChard');
responseRHC.output=GantryCraneOutput;
responseRHC.input=GantryCraneInput;
%% visualise the performance:
GantryResponsePlot(responseRHC.output.time,responseRHC.input.signals.values,...
    responseRHC.output.signals.values,[-1 -1],[1 1],[0 0],[xRange(2) yRange(2)],...
    [1 3],xTarget,'Nonlinear simulation: MPC performance');
%% plot trace of the load to check square tracking
figure;
scatter(responseRHC.output.signals.values(:,1)+stringLength*sin(responseRHC.output.signals.values(:,5)),...
    responseRHC.output.signals.values(:,3)-stringLength*sin(responseRHC.output.signals.values(:,7)))
hold on
scatter(responseRHC.output.signals.values(:,1), responseRHC.output.signals.values(:,3),'g')
scatter(xLow,yLow, 'r');
scatter(xLow,yHigh, 'r');
scatter(xHigh,yHigh, 'r');
scatter(xHigh,yLow, 'r');
