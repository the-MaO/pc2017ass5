function [Dt,Et,bt]=myStageConstraints(A,B,D,cl,ch,ul,uh)

DA = D*A;
DB = D*B;
m = size(DB,2);
n = size(A, 1);
I = eye(m);
z = zeros(m,n);

Dt = vertcat(DA, -DA, z, z);

Et = vertcat(DB, -DB, I, -I);

bt = vertcat(ch, -cl, uh, -ul);

end

