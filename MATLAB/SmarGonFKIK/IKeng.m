% SmarGon IK model, bare minimum  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This script is a simplification of the more complete IKsmargon Script
% It tries to do away with all fancy functions, and should work on
% rather basic infrastructure.
%
% 22.2.2018 Wayne Glettig

clear all;
%% USER Coordinates
X = 1e-3;
Y = 1e-3;
Z = 0;
CHI = 0;
PHI = 0;
OMEGA = 0*pi/180;
%theta =0;
SHX=0;
SHY=0;
SHZ=0;


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


%% the FK model is:

% X= l01 + l23 + l32 + l41 + q3 + SHX*cos(theta) + l74*cos(theta) + l71*sin(theta) - SHY*cos(q5)*sin(theta) + SHZ*sin(q5)*sin(theta);
% Y= l31*cos(q6) - SHZ*(cos(q5)*sin(q6) + cos(q6)*cos(theta)*sin(q5)) - SHY*(sin(q5)*sin(q6) - cos(q5)*cos(q6)*cos(theta)) - l42*cos(q6) + q1*cos(q6) - q2*sin(q6) + SHX*cos(q6)*sin(theta) - l71*cos(q6)*cos(theta) + l74*cos(q6)*sin(theta);
% Z= SHY*(cos(q6)*sin(q5) + cos(q5)*cos(theta)*sin(q6)) + SHZ*(cos(q5)*cos(q6) - cos(theta)*sin(q5)*sin(q6)) + q2*cos(q6) + l31*sin(q6) - l42*sin(q6) + q1*sin(q6) + SHX*sin(q6)*sin(theta) - l71*cos(theta)*sin(q6) + l74*sin(q6)*sin(theta);
% OMEGA= (180*q6)/pi;
% CHI= (180*theta)/pi; 
% PHI= (180*q5)/pi;


%% The IK model using the Pseudo-inverse of the Jacobian matrix 
% to converge a solution towards a solution
% q are motor coords, x are user coords)

xt=[X;Y;Z;OMEGA;CHI;PHI]; %SET TARGET USER COORDS (X,Y,Z,OMEGA,CHI,PHI)

qc=[1e-3;1e-3;0;0;0;0]; %motor start values (q1,q2,q3,theta,q5,q6)
loopcond = 1;
loopcounter=0;
maxloops=30;
while loopcond
    %get current x values based on q 
    xc(1,1)= l01 + l23 + l32 + l41 + q3 + SHX*cos(qc(4)) + l74*cos(qc(4)) + l71*sin(qc(4)) - SHY*cos(qc(5))*sin(qc(4)) + SHZ*sin(qc(5))*sin(qc(4));
    xc(2,1)= l31*cos(qc(6)) - SHZ*(cos(qc(5))*sin(qc(6)) + cos(qc(6))*cos(qc(4))*sin(qc(5))) - SHY*(sin(qc(5))*sin(qc(6)) - cos(qc(5))*cos(qc(6))*cos(qc(4))) - l42*cos(qc(6)) + q1*cos(qc(6)) - q2*sin(qc(6)) + SHX*cos(qc(6))*sin(qc(4)) - l71*cos(qc(6))*cos(qc(4)) + l74*cos(qc(6))*sin(qc(4));
    xc(3,1)= SHY*(cos(qc(6))*sin(qc(5)) + cos(qc(5))*cos(theta)*sin(qc(6))) + SHZ*(cos(qc(5))*cos(qc(6)) - cos(theta)*sin(qc(5))*sin(qc(6))) + q2*cos(qc(6)) + l31*sin(qc(6)) - l42*sin(qc(6)) + q1*sin(qc(6)) + SHX*sin(qc(6))*sin(qc(4)) - l71*cos(qc(4))*sin(qc(6)) + l74*sin(qc(6))*sin(qc(4));
    xc(4,1)= (180*qc(6))/pi;
    xc(5,1)= (180*qc(4))/pi; 
    xc(6,1)= (180*qc(5))/pi;
    
    deltax=xt-xc; %get x error (target - current)
    if abs(deltax)<1e-9 %if abs error small enough, get out of loop
        loopcond=0;
        disp('solution found')
    end
    %Calculate the Jacobian:
    J= [       0,        0, 1,                                         l71*cos(theta) - SHX*sin(theta) - l74*sin(theta) - SHY*cos(qc(5))*cos(theta) + SHZ*cos(theta)*sin(qc(5)),                                                         SHZ*cos(qc(5))*sin(theta) + SHY*sin(qc(5))*sin(theta),                                                                                                                                                                                                                                        0;...
         cos(qc(6)), -sin(qc(6)), 0, SHX*cos(qc(6))*cos(theta) + l74*cos(qc(6))*cos(theta) + l71*cos(qc(6))*sin(theta) - SHY*cos(qc(5))*cos(qc(6))*sin(theta) + SHZ*cos(qc(6))*sin(qc(5))*sin(theta), SHZ*(sin(qc(5))*sin(qc(6)) - cos(qc(5))*cos(qc(6))*cos(theta)) - SHY*(cos(qc(5))*sin(qc(6)) + cos(qc(6))*cos(theta)*sin(qc(5))), l42*sin(qc(6)) - SHZ*(cos(qc(5))*cos(qc(6)) - cos(theta)*sin(qc(5))*sin(qc(6))) - q2*cos(qc(6)) - l31*sin(qc(6)) - SHY*(cos(qc(6))*sin(qc(5)) + cos(qc(5))*cos(theta)*sin(qc(6))) - q1*sin(qc(6)) - SHX*sin(qc(6))*sin(theta) + l71*cos(theta)*sin(qc(6)) - l74*sin(qc(6))*sin(theta);...
         sin(qc(6)),  cos(qc(6)), 0, SHX*cos(theta)*sin(qc(6)) + l74*cos(theta)*sin(qc(6)) + l71*sin(qc(6))*sin(theta) - SHY*cos(qc(5))*sin(qc(6))*sin(theta) + SHZ*sin(qc(5))*sin(qc(6))*sin(theta), SHY*(cos(qc(5))*cos(qc(6)) - cos(theta)*sin(qc(5))*sin(qc(6))) - SHZ*(cos(qc(6))*sin(qc(5)) + cos(qc(5))*cos(theta)*sin(qc(6))), l31*cos(qc(6)) - SHZ*(cos(qc(5))*sin(qc(6)) + cos(qc(6))*cos(theta)*sin(qc(5))) - SHY*(sin(qc(5))*sin(qc(6)) - cos(qc(5))*cos(qc(6))*cos(theta)) - l42*cos(qc(6)) + q1*cos(qc(6)) - q2*sin(qc(6)) + SHX*cos(qc(6))*sin(theta) - l71*cos(qc(6))*cos(theta) + l74*cos(qc(6))*sin(theta);...
               0,        0, 0,                                                                                                                                          0,                                                                                                       0,                                                                                                                                                                                                                                   180/pi;...
               0,        0, 0,                                                                                                                                     180/pi,                                                                                                       0,                                                                                                                                                                                                                                        0;...
               0,        0, 0,                                                                                                                                          0,                                                                                                  180/pi,                                                                                                                                                                                                                                        0];
    Jinvc = pinv(J);
    deltaq=Jinvc*deltax; %By multiplying the x error with Jinv, a q correction can be deduced
    qc = qc+deltaq;%update current motor values

    loopcounter=loopcounter+1;
    if loopcounter > maxloops
        loopcond = 0;
        disp('30 iterations reached')
    end
end
q = qc %output q as the motor coordinates

%% The IK model using the Pseudo-inverse of the Jacobian matrix 
% to converge a solution towards a solution
 
% Jacobian of FK f function derived over motor coordinates



J= jacobian(f,[q1,q2,q3,theta,q5,q6]); % SO FAR ANALYTICAL. FROM HERE ON NUMERICAL.

Jinv = pinv(J);
 
tic
% Algorithm for IK (q are motor coords, x are user coords)
xt=[0.1687391293583430989690637491127;-0.015209423663512846293490651681675;0.002;0;47.990543436670047241427846041931;0.2865]; %SET TARGET USER COORDS 
q0=[1e-3;1e-3;0;0;0;0]; %motor start values
qc = q0; %set current q values for loop
loopcond = 1;
loopcounter=0;
while loopcond
    xc = vpa(subs(f, [q1,q2,q3,theta,q5,q6], qc')); %get current x values based on q 
    deltax=xt-xc; %get x error (target - current)
    if abs(deltax)<1e-9 %if abs error small enough, get out of loop
        loopcond=0;
        disp('error small enough')
    end
    Jinvc=vpa(subs(Jinv, [q1,q2,q3,theta,q5,q6], qc')); %inv Jacobian with current q
    deltaq=Jinvc*deltax; %By multiplying the x error with Jinv, a q correction can be deduced
    qc = qc+deltaq;%update current motor values
    loopcounter=loopcounter+1;
    if loopcounter > 30
        loopcond = 0;
        disp('30 iterations reached')
    end
end
q = qc %output q as the motor coordinates
toc
 
%% Calculate q4 based on q3 and theta
% % H35 has q4 in it, which must be calculated.
% % Distance between P560in5 and P770in5
P560in5 = [l51;-l52;0;1];
P770in7 = [-l72;-l71-l73;0;1];
% we need H35 = inv(H53)
H35=inv(H53);
P770in5 = H35*H43*H74*P770in7;
P770in5= subs(P770in5, [q1,q2,q3,theta,q5,q6], qc')
eqn = sqrt((P770in5(1)-P560in5(1))^2+(P770in5(2)-P560in5(2))^2+(P770in5(3)-P560in5(3))^2) - l61;
q4_calc = vpasolve(eqn,q4,0); %Look more into this. 
% H74 = vpa(subs(H74,theta,th));
