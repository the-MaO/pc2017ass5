clear variables
close all
clc
%% SimscapeCrane_MPC_start;
load('Params_Simscape.mat');
load('SSmodelParams.mat');
%% Load the dynamics matrices using a solution from last assignment
Ts=1/30; % inc. improves non-linear performance, also increases computation time
% dec just shit
[A,B,C,~] = genCraneODE(m,M,MR,r,g,Tx,Ty,Vm,Ts);

%% Define other parameters
N=ceil(1/Ts); % ceiling to ensure N is an integer
    % inc improves slightly, inc comp time
T=20;

% constant to correct for "stickiness" of the crane in X axis
stickCorr = 0.01;

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

% define constraints for each side
margin = 0.1;
scl1 = [xHigh - margin; yLow];
scl2 = [xLow; yHigh - margin];
scl3 = [xLow - margin; yLow];
scl4 = [xLow; yLow - margin];

sch1 = [xHigh + margin; yHigh];
sch2 = [xHigh; yHigh + margin];
sch3 = [xLow + margin; yHigh];
sch4 = [xHigh; yLow + margin];
% scl1 = [0; 0];
% scl2 = [0; 0];
% scl3 = [0; 0];
% scl4 = [0; 0];
% 
% sch1 = [1; 1];
% sch2 = [1; 1];
% sch3 = [1; 1];
% sch4 = [1; 1];
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
angleConstraint=2*pi/180; % in radians
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
[Dt1,Et1,bt1]=myStageConstraints(A,B,D,cl1,ch1,ul,uh);
[Dt2,Et2,bt2]=myStageConstraints(A,B,D,cl2,ch2,ul,uh);
[Dt3,Et3,bt3]=myStageConstraints(A,B,D,cl3,ch3,ul,uh);
[Dt4,Et4,bt4]=myStageConstraints(A,B,D,cl4,ch4,ul,uh);


%% Compute trajectory constraints matrices and vector
[DD1,EE1,bb1]=myTrajectoryConstraints(Dt1,Et1,bt1,N);
[DD2,EE2,bb2]=myTrajectoryConstraints(Dt2,Et2,bt2,N);
[DD3,EE3,bb3]=myTrajectoryConstraints(Dt3,Et3,bt3,N);
[DD4,EE4,bb4]=myTrajectoryConstraints(Dt4,Et4,bt4,N);

%% Compute QP constraint matrices
[Gamma,Phi] = genPrediction(A,B,N); % get prediction matrices:

[F,J,L]=myConstraintMatrices(DD1,EE1,Gamma,Phi,N);


%% Compute QP cost matrices
[H,G] = genCostMatrices(Gamma,Phi,Q,R,P,N);

%% Prepare cost and constraint matrices for mpcqpsolver
% Calculating the inverse of the lower triangular H. see doc mpcqpsolver.
[H,p] = chol(H,'lower');
H=(H'\eye(size(H)))';

%% create arrays of combined constraint matrices for 4 corners
bb = [bb1, bb2, bb3, bb4];

%% Running a matlab simulation and visualisng the results:
MatlabSimulation
GantryResponsePlot(t,allU,...
    x,[-1 -1],[1 1],[0 0],[xRange(2) yRange(2)],[1 3],xTarget,'Linear simulation: MPC performance');
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
scatter(responseRHC.output.signals.values(:,1),responseRHC.output.signals.values(:,3))
hold on
scatter(xLow,yLow, 'r');
scatter(xLow,yHigh, 'r');
scatter(xHigh,yHigh, 'r');
scatter(xHigh,yLow, 'r');