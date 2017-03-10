% Testing script for Assignment 3 functions.
addpath('../assignment 1')
addpath('../assignment 2')

% 0 to choose not to test. 1 to select.
choices = [1, 1, 1, 0];

% Prepare some reused stuff.
A = magic(3);
B = randn(3, 2);
D = randn(4, 3);

cl = ones(4, 1);
ch = ones(4, 1) * 4;
ul = ones(2, 1) * 0.5;
uh = ones(2, 1) * 5;

% Test for myStageConstraints.m
if choices(1) == 1    
    [Dt,Et,bt] = myStageConstraints(A, B, D, cl, ch, ul, uh);
    
    Dt_correct = [D * A; -(D * A); zeros(4, 3)];
    Et_correct = [D * B; -(D * B); eye(2); -1 * eye(2)];
    bt_correct = [4; 4; 4; 4; -1; -1; -1; -1; 5; 5; -.5; -.5];
    
    wrong1 = sum(sum(round(Dt_correct, 8) ~= round(Dt, 8))) + ...
        sum(sum(round(Et_correct, 8) ~= round(Et, 8))) + ...
        sum(sum(round(bt_correct, 8) ~= round(bt, 8)));
    
    if wrong1 > 0
        disp('Test for myStageConstraints.m failed.');
        disp('Wrong values: ');
        disp(wrong1);
    else
        disp('Test for myStageConstraints.m passed.');
    end
    
end

% Test for myTrajectoryConstraints.m
if choices(2) == 1    
    [Dt,Et,bt] = myStageConstraints(A, B, D, cl, ch, ul, uh);
    N = 3;
    [DD,EE,bb] = myTrajectoryConstraints(Dt, Et, bt, N);
    
    zrd = zeros(12, 3);
    DD_correct = [Dt, zrd, zrd; zrd, Dt, zrd; zrd, zrd, Dt];
    
    zre = zeros(12, 2);
    EE_correct = [Et, zre, zre; zre, Et, zre; zre, zre, Et];
    
    bb_correct = [bt; bt; bt];
    
    wrong2 = sum(sum(round(DD_correct, 8) ~= round(DD, 8))) + ...
        sum(sum(round(EE_correct, 8) ~= round(EE, 8))) + ...
        sum(sum(round(bb_correct, 8) ~= round(bb, 8)));
    
    if wrong2 > 0
        disp('Test for myTrajectoryConstraints.m failed.');
        disp('Wrong values: ');
        disp(wrong2);
    else
        disp('Test for myTrajectoryConstraints.m passed.');
    end
end

% Test for myConstraintMatrices.m
if choices(3) == 1
    [Dt,Et,bt] = myStageConstraints(A, B, D, cl, ch, ul, uh);
    N = 3;
    [DD,EE,bb] = myTrajectoryConstraints(Dt, Et, bt, N);
    [Gamma,Phi] = myPrediction(A, B, N);
    [F, J, L] = myConstraintMatrices(DD, EE, Gamma, Phi, N);
    
    Gamma_tilda = [zeros(3, 6); Gamma(1:6, :)];
    Phi_tilda = [eye(3); Phi(1:6, :)];
    
    F_correct = (DD * Gamma_tilda) + EE;
    J_correct = -1 * (DD * Phi_tilda);
    L_correct = (-1 * J) - (DD * [eye(3); eye(3); eye(3)]);
    
    wrong3 = sum(sum(round(F_correct, 8) ~= round(F, 8))) + ...
        sum(sum(round(J_correct, 8) ~= round(J, 8))) + ...
        sum(sum(round(L_correct, 8) ~= round(L, 8)));
    
    if wrong3 > 0
        disp('Test for myConstraintMatrices.m failed.');
        disp('Wrong values: ');
        disp(wrong3);
    else
        disp('Test for myConstraintMatrices.m passed.');
    end
end

% Test for myMPController.m
if choices(4) == 1
end