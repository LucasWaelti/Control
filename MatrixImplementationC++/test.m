%% Test Script
% The same operation are implemented in C++. 
A = [1,2,2;3,1,1;2,2,1]
B = inv(A)
C = A + B
D = B * C
E = D'
F = E*2
G = ((A+B)*inv(C)-E)/2