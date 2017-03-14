function [F,J,L]=myConstraintMatrices(DD,EE,Gamma,Phi,N)

n = size(Phi, 2);
m = size(Gamma, 2)/N;

PhiB = vertcat (eye(n), Phi(1:n*(N-1), :));

GammaB = vertcat (zeros(n,N*m), Gamma(1:n*(N-1), :));

F = DD * GammaB + EE;
J = -DD*PhiB;
L = -J - DD*(kron(ones(N,1),eye(n)));

end

