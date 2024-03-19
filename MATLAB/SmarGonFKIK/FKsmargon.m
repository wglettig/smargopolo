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
     alpha beta gamma theta;
%    q1 q2 q3 q4 q5 q6;


%% MOTOR Coordinates
q1 = 0.0190;
q2 = -0.0273;
q3 = 0.0255;
q4 = 0.0107;
q5 = 0;
q6 = 0;
%theta =0;
SHX=0;
SHY=0;
SHZ=27.35e-3; 


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
%Rot around Y:
       0,0        ,0           ,1];
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
l01 = 42.5e-3;
l11 = 25e-3 - (17e-3)/2; %half distance between sliders table midline
l12 = l11;
l21 = l11;
l22 = l11;
l23 = 13.5e-3; %Distance between q1 & q2 stage level
l31 = 11.5e-3; %Distance from q3 table to middle of red part
l32 = 68.5e-3 - (80e-3)/2;
l33 = l31;
l34 = l32;
l41 = 76.5e-3;
l42 = 25.5e-3;
l51 = 10e-3;
l52 = 2.5e-3; 
l61 = 64.422e-3; % Connecting rod length
l71 = 5e-3;  % Swing dimensions
l72 = 17.67e-3;
l73 = 5.2e-3;
l74 = 1.53e-3;


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
H74 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [l41,-l42,0,0,0,theta]);
H87 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [l74,-l71,0,q5,0,0]);




%% The FK model (forward kinematics) can be defined as 
% [X;Y;Z;OMEGA;CHI;PHI] = f(q1,q2,q3,q4,q5,q6,SHX,SHY,SHZ)

% H74 has theta in it, which must be calculated.
% Distance between P560in5 and P770in5
P560in5 = [l51;-l52;0;1];
P770in7 = [-l72;-l71-l73;0;1];
% we need H35 = inv(H53)
H35=inv(H53);
P770in5 = H35*H43*H74*P770in7;

eqn = sqrt((P770in5(1)-P560in5(1))^2+(P770in5(2)-P560in5(2))^2+(P770in5(3)-P560in5(3))^2) - l61;
th = vpasolve(eqn,theta,0); %Look more into this. 
H74 = vpa(subs(H74,theta,th));

% Output point (P800)
P800in8 = [SHX;SHY;SHZ;1];
H80 = H10*H21*H32*H43*H74*H87;  
P800in0 = H80*P800in8;
X=P800in0(1)
Y=P800in0(2)
Z=P800in0(3)
OMEGA = q6*180/pi
CHI = vpa(th*180/pi)
PHI = q5*180/pi



%% Plot the results (construct sample position using motors)
Oin1 = [0;0;0;1]; 
e1in1 = [1;0;0;1];
e2in1 = [0;1;0;1];
e3in1 = [0;0;1;1];

%clf;
hold on;
grid on;
axis vis3d;
%axis([-10 10 -10 10 -10 10]);
title('SmarGon Direct Model (q1,q2,q3,q4,q5)');
xlabel('Beamline Z-Axis');
ylabel('Beamline X-Axis');
zlabel('Beamline Y-Axis');

% Build Base (Black part)
P000in0 = [0;0;0;1];
P100in0 = [l01;0;0;1];
P000inM = H0M*P000in0;
P100inM = H0M*P100in0;
plot3([P000inM(1),P100inM(1)],[P000inM(2),P100inM(2)],[P000inM(3),P100inM(3)],'k-');

% Build Blue part, RF1
P100in1 = [0;0;0;1];
P110in1 = [0;0;l11;1];
P111in1 = [0;0;-l11;1];
P100inM = H0M*H10*P100in1;
P110inM = H0M*H10*P110in1;
P111inM = H0M*H10*P111in1;
plot3([P110inM(1),P100inM(1),P111inM(1)],[P110inM(2),P100inM(2),P111inM(2)],[P110inM(3),P100inM(3),P111inM(3)],'b+-');


% Build Green part, RF2
P210in2 = [0;0;l21;1];
P211in2 = [0;0;-l21;1];
P213in2 = [0;0;0;1];
P220in2 = [l23;l31;0;1];
P221in2 = [l23;-l31;0;1];
P223in2 = [l23;0;0;1];
P210inM = H0M*H10*H21*P210in2;
P211inM = H0M*H10*H21*P211in2;
P213inM = H0M*H10*H21*P213in2;
P220inM = H0M*H10*H21*P220in2;
P221inM = H0M*H10*H21*P221in2;
P223inM = H0M*H10*H21*P223in2;
plot3([P210inM(1),P213inM(1),P211inM(1)],[P210inM(2),P213inM(2),P211inM(2)],[P210inM(3),P213inM(3),P211inM(3)],'g+-');
plot3([P213inM(1),P223inM(1)],[P213inM(2),P223inM(2)],[P213inM(3),P223inM(3)],'g+-');
plot3([P220inM(1),P223inM(1),P221inM(1)],[P220inM(2),P223inM(2),P221inM(2)],[P220inM(3),P223inM(3),P221inM(3)],'g+-');

% Build Red part, RF3
P320in3 = [0;l31;0;1];
P321in3 = [0;-l33;0;1];
P323in3 = [0;0;0;1];
P330in3 = [l32;l31;0;1];
P340in3 = [l34;-l33;0;1];
P320inM = H0M*H10*H21*H32*P320in3;
P321inM = H0M*H10*H21*H32*P321in3;
P323inM = H0M*H10*H21*H32*P323in3;
P330inM = H0M*H10*H21*H32*P330in3;
P340inM = H0M*H10*H21*H32*P340in3;
plot3([P320inM(1),P323inM(1),P321inM(1)],[P320inM(2),P323inM(2),P321inM(2)],[P320inM(3),P323inM(3),P321inM(3)],'r+-');
plot3([P320inM(1),P330inM(1)],[P320inM(2),P330inM(2)],[P320inM(3),P330inM(3)],'r+-');
plot3([P321inM(1),P340inM(1)],[P321inM(2),P340inM(2)],[P321inM(3),P340inM(3)],'r+-');

% Build Orange part, RF4
P430in4 = [0;0;0;1];
P440in4 = [l41;0;0;1];
P450in4 = [l41;-l42;0;1];
P430inM = H0M*H10*H21*H32*H43*P430in4;
P440inM = H0M*H10*H21*H32*H43*P440in4;
P450inM = H0M*H10*H21*H32*H43*P450in4;
plot3([P430inM(1),P440inM(1),P450inM(1)],[P430inM(2),P440inM(2),P450inM(2)],[P430inM(3),P440inM(3),P450inM(3)],'+-', 'color',[250/255,140/255,0]);

% Build Olive part, RF5
P540in5 = [0;0;0;1];
P550in5 = [l51;0;0;1];
P560in5 = [l51;-l52;0;1];
P540inM = H0M*H10*H21*H32*H53*P540in5;
P550inM = H0M*H10*H21*H32*H53*P550in5;
P560inM = H0M*H10*H21*H32*H53*P560in5;
plot3([P540inM(1),P550inM(1),P560inM(1)],[P540inM(2),P550inM(2),P560inM(2)],[P540inM(3),P550inM(3),P560inM(3)],'+-', 'color',[122/255,108/255,0]);


% Build Yellow part, RF7
P750in7 = [0;0;0;1];
P751in7 = [0;-l71;0;1];
P770in7 = [-l72;-l71-l73;0;1];
P771in7 = [-l72;-l71;0;1];
P780in7 = [l74;-l71;0;1];

P750inM = H0M*H10*H21*H32*H43*H74*P750in7;
P751inM = H0M*H10*H21*H32*H43*H74*P751in7;
P770inM = H0M*H10*H21*H32*H43*H74*P770in7;
P771inM = H0M*H10*H21*H32*H43*H74*P771in7;
P780inM = H0M*H10*H21*H32*H43*H74*P780in7;

%% plot it
plot3([P750inM(1),P751inM(1),P771inM(1),P770inM(1)],[P750inM(2),P751inM(2),P771inM(2),P770inM(2)],[P750inM(3),P751inM(3),P771inM(3),P770inM(3)],'+-', 'color',[255/255,255/255,128/255]);
plot3([P751inM(1),P780inM(1)],[P751inM(2),P780inM(2)],[P751inM(3),P780inM(3)],'+-', 'color',[255/255,255/255,128/255]);


%Distance between P560inM and P770inM
d= sqrt((P770inM(1)-P560inM(1))^2+(P770inM(2)-P560inM(2))^2+(P770inM(3)-P560inM(3))^2)
l61




