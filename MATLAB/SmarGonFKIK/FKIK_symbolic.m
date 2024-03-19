% FK/IK using homogenous matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This script demonstrates how homogenous matrices can be used to create
% both a forward kinematics (FK)and an inverse kinematics (IK) model of 
% a serial kinematics manipulator, with 5 % motors (q1,q2,q3,q4,q5) 
% and 4 user coordinates (X,Y,Z,theta).
%
% The script uses a chain of Homogenous Transformation Matrices to create 
% the FK model. And the Moore-Penrose Pseudo-Inverse of the Jacobian 
% to iteratively solve the IK model (Newton-Raphson).
%
% 5.10.2017 Wayne Glettig
  
clear all;
syms offsetX offsetY offsetZ ...
     alpha beta gamma theta ...
     q1 q2 q3 q4 q5 q6 ...
     l01 ...
     l11 l12 ...
     l21 l22 l23...
     l31 l32 l33 l34...
     l41 l42...
     l51 l52...
     l61...
     l71 l72 l73 l74...
     SHX SHY SHZ;



%% USER Coordinates
X = 1e-3;
Y = 1e-3;
Z = 0;
CHI = 0;
PHI = 0;
OMEGA = 0*pi/180;
%theta =0;

%% Here the Transformation Matrix HBA is constructed from a translation 
% matrix and 3 rotation matrices, using symbolic variables.
%Translation:
HXYZ = [1,0,0,offsetX;...
        0,1,0,offsetY;...
        0,0,1,offsetZ;...
        0,0,0,1];
%Rot around X:
HRX = [1,0        ,0           ,0;...
       0,cos(alpha),-sin(alpha),0;...
       0,sin(alpha), cos(alpha),0;...
       0,0        ,0           ,1];
%Rot around Y:
HRY = [ cos(beta),0,sin(beta),0;...
       0         ,1,0        ,0;...
       -sin(beta),0,cos(beta),0;...
       0         ,0,0        ,1];
%Rot around Z:
HRZ = [cos(gamma),-sin(gamma),0,0;...
       sin(gamma), cos(gamma),0,0;...
       0         ,0          ,1,0;...
       0         ,0          ,0,1];
%Create HBA (first rotation then translation)
HBA=HXYZ*HRZ*HRY*HRX;
%Create HAB (inverse)
HAB=inv(HBA);

%% Functional lengths
% l01 = 42.5e-3;
% l11 = 25e-3 - (17e-3)/2; %half distance between sliders table midline
% l12 = l11;
% l21 = l11;
% l22 = l11;
% l23 = 13.5e-3; %Distance between q1 & q2 stage level
% l31 = 11.5e-3; %Distance from q3 table to middle of red part
% l32 = 68.5e-3 - (80e-3)/2;
% l33 = l31;
% l34 = l32;
% l41 = 76.5e-3;
% l42 = 25.5e-3;
% l51 = 10e-3;
% l52 = 2.5e-3;
% l61 = 64.422e-3; % Connecting rod length
% l71 = 5e-3;  % Swing dimensions
% l72 = 17.67e-3;
% l73 = 5.2e-3;
% l74 = 1.53e-3;


%% Here the Kinematic Chain is created (q1,q2,q3,q4,q5 are motor coordinates)
%Essentially the HBA matrix above is filled with the offsetXYZ and angle
%values to assemble the chain. The chain consists here of the reference
%frames 0-1-2-3-4-5-6-7:
%Frame M: MATLAB Plot Frame (Z up)
%Frame 0: On Aerotech, before Omega
%Frame 1: On Blue Part
%Frame 2: On Green Part
%Frame 3: On Red Part (T-Part)
%Frame 4: On Orange Part
%Frame 5: On Olive Part
%Frame 6: On Purple Part (Connecting rod)
%Frame 7: On Yellow Part (Swing)
%Frame 8: On Pink part (Sample pin)

H0M = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [0,0,0,pi/2,0,pi/2]);
H10 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [l01,0,0,q6,0,0]);
H21 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [0,q1,0,0,0,0]);
H32 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [l23,0,q2,0,0,0]);
H43 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [l32+q3,l31,0,0,0,0]);
H53 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [l34+q4,-l33,0,0,0,0]);
H35 = inv(H53);
H74 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [l41,-l42,0,0,0,theta]);
H87 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [l74,-l71,0,q5,0,0]);




%% The FK model (forward kinematics) can be defined as 
% [X;Y;Z;OMEGA;CHI;PHI] = f(q1,q2,q3,q4,q5,q6,SHX,SHY,SHZ)

% % H74 has theta in it, which must be calculated.
% % Distance between P560in5 and P770in5
P560in5 = [l51;-l52;0;1];
P770in7 = [-l72;-l71-l73;0;1];
% we need H35 = inv(H53)
H35=inv(H53);
P770in5 = H35*H43*H74*P770in7;
eqn = sqrt((P770in5(1)-P560in5(1))^2+(P770in5(2)-P560in5(2))^2+(P770in5(3)-P560in5(3))^2) - l61;
%th = vpasolve(eqn,theta,0); %Look more into this. 
% H74 = vpa(subs(H74,theta,th));

% Output point (P800)
P800in8 = [SHX;SHY;SHZ;1];
H80 = H10*H21*H32*H43*H74*H87;  
P800in0 = H80*P800in8;
X=P800in0(1);
Y=P800in0(2);
Z=P800in0(3);
OMEGA = q6*180/pi;
CHI = theta*180/pi;
PHI = q5*180/pi;

f(1,1) = X;
f(2,1) = Y;
f(3,1) = Z;
f(4,1) = OMEGA;
f(5,1) = CHI;
f(6,1) = PHI;

%%Get jacobian

J= jacobian(f,[q1,q2,q3,theta,q5,q6]); % SO FAR ANALYTICAL. FROM HERE ON NUMERICAL.
Jpinv = pinv(J); %Moore-Penrose Pseudoinverse

%% Calculate q4 based on q3 and theta
% % H35 has q4 in it, which must be calculated.
% % Distance between P560in5 and P770in5
P560in5 = [l51;-l52;0;1];
P770in7 = [-l72;-l71-l73;0;1];
% we need H35 = inv(H53)
H35=inv(H53);
P770in5 = H35*H43*H74*P770in7;
eqn = sqrt((P770in5(1)-P560in5(1))^2+(P770in5(2)-P560in5(2))^2+(P770in5(3)-P560in5(3))^2) - l61;
eqn_d_theta = diff(eqn, theta)
eqn_d_q4 = diff(eqn, q4)

