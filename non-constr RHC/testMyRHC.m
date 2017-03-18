clear variables
close all
%% Load the parameters
load('Params_Simscape.mat');
load('SSmodelParams.mat');
%% Declare simulation parameters
Ts=1/30;    % sampling time
N=20;  %short horizon makes slow response
T=50;
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

[A,B,C,D] = genCraneODE(m,M,MR,r,g,Tx,Ty,Vm,Ts);
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
%% Compose prediction matrices for RHC
% your myPrediction function is called here and the variables Gamma and Phi are 
% declared in the workspace. 
[Gamma,Phi]=myPrediction(A,B,N);
%% Declare RHC control law
% The linear control law K is declared here. It will be visible to a Simulink 
% constant block and can be used to implement the control law.
% See how this is implemented here: SimscapeCrane_RHC/Controllers
[H,G] = myCostMatrices(Gamma,Phi,Q,R,P,N);
K = myRHC(H,G,size(B,2));
%% Run the simulations for your controller and the PID controller
% Select controller, Uncomment as required
% controlCase=1; % your RHC 
% controlCase=2; % P(ID)controller

% Open the model
SimscapeCrane_RHC; 

% controlCase=2;
% sim('SimscapeCrane_RHC');
% responsePP.output=GantryCraneOutput;
% responsePP.input=GantryCraneInput;

controlCase=1;
sim('SimscapeCrane_RHC');
responseRHC.output=GantryCraneOutput;
responseRHC.input=GantryCraneInput;

%% visualise the performance:
help GantryResponsePlot
% GantryResponsePlot(responsePP.output.time,responsePP.input.signals.values,...
%     responsePP.output.signals.values,[-1 -1],[1 1],[0 0],[xRange(2) yRange(2)],[1 3],xTarget,'PID performance');
GantryResponsePlot(responseRHC.output.time,responseRHC.input.signals.values,...
    responseRHC.output.signals.values,[-1 -1],[1 1],[0 0],[xRange(2) yRange(2)],[1 3],xTarget1,'RHC performance');

%% plot trace of the load to check square tracking
stringLength = 0.5;
craneMovementPlot(responseRHC.output.signals.values(:,1),responseRHC.output.signals.values(:,3),...
    responseRHC.output.signals.values(:,5),responseRHC.output.signals.values(:,7),...
    xLow,xHigh,yLow,yHigh,zeros(2,8),stringLength,'Non-constrained simulation');

%% calculate squarness of the square tracked
% WARNING this calculation is a function of the string length. As the
% lenght is estimated, the calculation is not not numerically accurate,
% however, in comparison between squares it is informative.

% get the traces of the pendulum
x_pend = responseRHC.output.signals.values(:,1) + stringLength* sin(responseRHC.output.signals.values(:,5));
y_pend = responseRHC.output.signals.values(:,3) + stringLength* sin(responseRHC.output.signals.values(:,7));

% get the coordinates of the largest circumscribed square
x_min = min(x_pend);
x_max = max(x_pend);
y_min = min(y_pend);
y_max = max(y_pend);
hline(y_min,'c');
hline(y_max,'c');
vline(x_min,'c');
vline(x_max,'c');

% get coordinates of largest inscribed square
% find x-coordinates of trace left and right side
side_log1 = y_pend > 1.15*yLow;
side_log2 = y_pend < 0.95*yHigh;
x_side = x_pend(side_log1 & side_log2);
% separate to left and right side
side_log1 = x_side < 0.3;
side_log2 = x_side > 0.3;
x_left = x_side(side_log1);
x_right = x_side(side_log2);
% get x-coordinates of inscribed square
x_ins_min = max(x_left);
x_ins_max = min(x_right);

vline(x_ins_max,'c');
vline(x_ins_min,'c');

% find y-coordinates of trace top and bottom side
side_log1 = x_pend > 1.15*xLow;
side_log2 = x_pend < 0.95*xHigh;
y_side = y_pend(side_log1 & side_log2);
% separate to left and right side
side_log1 = y_side < 0.3;
side_log2 = y_side > 0.3;
y_bot = y_side(side_log1);
y_top = y_side(side_log2);
% get y-coordinates of inscribed square
y_ins_min = max(y_bot);
y_ins_max = min(y_top);

hline(y_ins_max,'c');
hline(y_ins_min,'c');

% calculate space the trace needs
% the smaller this number, the better square is tracked
a_cir = (x_max - x_min)*(y_max - y_min);
a_ins = (x_ins_max - x_ins_min) * (y_ins_max - y_ins_min);

squarness = a_cir - a_ins