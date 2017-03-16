N = 4;
A = [2 1 3; 3 5 3; 5 3 1];
B = [3;9;5];

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
