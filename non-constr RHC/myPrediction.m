function [Gamma,Phi] = myPrediction(A,B,N)
% MYPREDICTION  [Gamma,Phi] = myPrediction(A,B,N). 
% A and B are discrete-time state space matrices for x[k+1]=Ax[k]+Bu[k]
% N is the horizon length. 
% Your code is suppose to work for any linear system, not just the gantry crane. 

% your code here:
Phi = A;

Gamma = kron(diag(ones(1,N),0),B);

for i=2:N
    lala = diag(ones(1,N+1-i),1-i);
    f = A^(i-1);
    f = f*B;
    lala = kron(lala,f);
    Gamma = Gamma + lala;
    
    p = A^i;
    Phi = [Phi;p];
end
end

