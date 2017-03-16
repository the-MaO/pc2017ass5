function [H,G] = myCostMatrices(Gamma,Phi,Q,R,P,N)
%% cost function matrices
% Gamma and Phi are the prediction matrices
% Q is the stage cost weight on the states, i.e. x'Qx
% R is the stage cost weight on the inputs, i.e. u'Ru
% P is the terminal weight on the final state

% your code here
R_dash = kron(eye(N),R);
Q_dash = kron(eye(N-1),Q);
Q_dash = [Q_dash zeros(size(Q_dash,1),size(P,2)); zeros(size(P,1),size(Q_dash,2)) P];

Gamma_t = Gamma.';

H = 2*(R_dash + Gamma_t*Q_dash*Gamma);
G = 2*(Gamma_t*Q_dash*Phi);

end