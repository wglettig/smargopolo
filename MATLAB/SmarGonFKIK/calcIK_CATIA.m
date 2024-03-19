function [b1,b2,b3,b4,b5,b6] = calcIK_CATIA(X,Y,Z,OMEGA,CHI,PHI)
%CALCIK_CATIA Summary of this function goes here
%   Detailed explanation goes here

[q1,q2,q3,q4,q5,q6] = calcIK(X,Y,Z,OMEGA,CHI,PHI)

% calcIK variables
% q1	slider in Y direction 
% q2	slider in Z direction 
% q3	upper x slider
% q4	lower x slider
% q5	Phi Rotation
% q6	Omega Rotation

% CATIA variables
% b1	Omega Rotation
% b2	slider in Y direction 
% b3	slider in Z direction 
% b4	upper x slider
% b5	lower x slider
% b6	Phi Rotation


b1 = q6 + 180;
b2 = -q1 -4.836e-3;
b3 = q2;
b4 = -q3 -6.837e-3;
b5 = -q4 -6.873e-3;
b6 = q5;

end

