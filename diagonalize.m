function [plant_ss_diag] = diagonalize(plant_ss)
%DIAGONALIZE Summary of this function goes here
%   Detailed explanation goes here

A = plant_ss.A; B = plant_ss.B; C = plant_ss.C;

[v,D] = eig(A);
A_d = D;
B_d = v \ B;
C_d = C * v;

plant_ss_diag = ss(A_d, B_d, C_d, 0);

end

