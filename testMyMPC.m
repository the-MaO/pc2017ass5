clear variables
% close all
clc
%% SimscapeCrane_MPC_start;
load('Params_Simscape.mat');
load('SSmodelParams.mat');
%% Load the dynamics matrices using a solution from last assignment
Ts=1/30;    % sampling time
[A,B,C,~] = genCraneODE(m,M,MR,r,g,Tx,Ty,Vm,Ts);

%% Define other parameters
N=20;       % horizon length
T=50;       % simulation time

% constant to correct for "stickiness" of the crane in X axis
stickCorr = 0.01;

% define sides of the square
xHigh = 0.45 + stickCorr;
xLow = 0.1 - stickCorr;
yHigh = 0.45;
yLow = 0.1;

% define target states
xTarget1=[xHigh 0 yHigh 0 0 0 0 0]';
xTarget2=[xLow 0 yHigh 0 0 0 0 0]';
xTarget3=[xLow 0 yLow 0 0 0 0 0]';
xTarget4=[xHigh 0 yLow 0 0 0 0 0]';

x0=[xHigh 0 yHigh 0 0 0 0 0]'; % starting offset

xZero = xHigh;
yZero = yHigh;

% define constraints for each side
margin = 0.003;
scl1 = [xHigh - margin; yLow];
scl2 = [xLow; yHigh - margin];
scl3 = [xLow - margin; yLow];
scl4 = [xLow; yLow - margin];

sch1 = [xHigh + margin; yHigh];
sch2 = [xHigh; yHigh + margin];
sch3 = [xLow + margin; yHigh];
sch4 = [xHigh; yLow + margin];

constraints = [scl1, scl2, scl3, scl4, sch1, sch2, sch3, sch4];
%% Declare penalty matrices and tune them here:
% current state cost matrix Q
Q=eye(8); 
              
Q(1,1) = 10;       
Q(2,2) = 1;
Q(3,3) = 10;
Q(4,4) = 1;
Q(5,5) = 50;
Q(6,6) = 2;
Q(7,7) = 50;
Q(8,8) = 2;

% input cost matrix R
R=eye(2)*0.01; 
   
% final state cost matrix P
P=eye(8);    
           
P(1,1) = 5;      
P(3,3) = 5;
P(5,5) = 30;
P(7,7) = 30;
%% Declare contraints
% Declaring constraints only on states (X,Y,theta,psi) and inputs u
angleConstraint=5*pi/180; % in radians

% create 4 sets of state constraints for 4 sides of the square
cl1=[scl1(1); scl1(2); -angleConstraint; -angleConstraint];
cl2=[scl2(1); scl2(2); -angleConstraint; -angleConstraint];
cl3=[scl3(1); scl3(2); -angleConstraint; -angleConstraint];
cl4=[scl4(1); scl4(2); -angleConstraint; -angleConstraint];

ch1=[sch1(1); sch1(2);  angleConstraint;  angleConstraint];
ch2=[sch2(1); sch2(2);  angleConstraint;  angleConstraint];
ch3=[sch3(1); sch3(2);  angleConstraint;  angleConstraint];
ch4=[sch4(1); sch4(2);  angleConstraint;  angleConstraint];

ul=[-1; -1];
uh=[1; 1];

% constrained vector is Dx, hence
D=zeros(4,8);D(1,1)=1;D(2,3)=1;D(3,5)=1;D(4,7)=1;

%% Compute stage constraint matrices and vector
[~,~,bt1]=myStageConstraints(A,B,D,cl1,ch1,ul,uh);
[~,~,bt2]=myStageConstraints(A,B,D,cl2,ch2,ul,uh);
[~,~,bt3]=myStageConstraints(A,B,D,cl3,ch3,ul,uh);
[Dt,Et,bt4]=myStageConstraints(A,B,D,cl4,ch4,ul,uh);

%% Compute trajectory constraints matrices and vector
[~,~,bb1]=myTrajectoryConstraints(Dt,Et,bt1,N);
[~,~,bb2]=myTrajectoryConstraints(Dt,Et,bt2,N);
[~,~,bb3]=myTrajectoryConstraints(Dt,Et,bt3,N);
[DD,EE,bb4]=myTrajectoryConstraints(Dt,Et,bt4,N);

%% Compute QP constraint matrices
[Gamma,Phi] = genPrediction(A,B,N); % get prediction matrices:

[F,J,L]=myConstraintMatrices(DD,EE,Gamma,Phi,N);

%% Compute QP cost matrices
[H,G] = genCostMatrices(Gamma,Phi,Q,R,P,N);

%% Prepare cost and constraint matrices for mpcqpsolver
% Calculating the inverse of the lower triangular H. see doc mpcqpsolver.
[H,p] = chol(H,'lower');
H=(H'\eye(size(H)))';

%% create arrays of combined constraint and target matrices for 4 corners
bb = bb1;       % bb must be vector, otherwise non-lin sim crashes
bb_all = [bb1, bb2, bb3, bb4];
xTarget_all = {xTarget1, xTarget2, xTarget3, xTarget4};

%% Running a matlab simulation and visualising the results:
% MatlabSimulation
% GantryResponsePlot(t,allU,...
%     x,[-1 -1],[1 1],[0 0],[xRange(2) yRange(2)],[1 3],xTarget1,'Linear simulation: MPC performance');

%% plot trace of the load to check square tracking
stringLength = 0.5;
% craneMovementPlot(x(:,1),x(:,3),x(:,5),x(:,7),xLow,xHigh,yLow,yHigh,constraints,stringLength,'Linear simulation');
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
    [1 3],xTarget1,'Nonlinear simulation: MPC performance');
%% plot trace of the load to check square tracking
craneMovementPlot(responseRHC.output.signals.values(:,1),...
    responseRHC.output.signals.values(:,3),responseRHC.output.signals.values(:,5),...
    responseRHC.output.signals.values(:,7),xLow,xHigh,yLow,yHigh,constraints,stringLength,'Non-linear simulation');

%% calculate squarness of the square tracked
% WARNING this calculation is a function of the string length. As the
% lenght is estimated, the calculation is not not numerically accurate,
% however, in comparison between squares it is informative.

x_pend = responseRHC.output.signals.values(:,1) + stringLength* sin(responseRHC.output.signals.values(:,5));
y_pend = responseRHC.output.signals.values(:,3) + stringLength* sin(responseRHC.output.signals.values(:,7));

x_min = min(x_pend);
x_max = max(x_pend);
y_min = min(y_pend);
y_max = max(y_pend);