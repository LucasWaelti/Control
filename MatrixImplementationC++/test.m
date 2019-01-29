%% Test Script
% The same operation are implemented in C++. 
A = [1,2,2;3,1,1;2,2,1]
B = inv(A)
C = A + B
D = B * C
E = D'
F = E*2
G = ((A+B)*inv(C)-E)/2

Phi = [2 1; -5 4];
Gamma = [0; 1];
Q1 = [1 0; 0 1];
Q2 = 1;
[K,~,~] = dlqr(Phi,Gamma,Q1,Q2)

n_iterations=1500;
for i=n_iterations:-1:1
                    
    %%- Ricatti Equation 
    R = Q2 + Gamma'*S*Gamma;
    M = S - S*Gamma*inv(R)*Gamma'*S;
   %K = ...
    S = Phi'*M*Phi + Q1;

    %%- store singular values (use svd function) 
    singular_values(i,:) = svd(S);
end
KLqrRiccati = inv(R)*Gamma'*S*Phi