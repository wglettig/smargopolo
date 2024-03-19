function [X,Y,Z,OMEGA,CHI,PHI] = calcFK(q1,q2,q3,q4,q5,q6)
% SmarGon IK model, bare minimum  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This script is a simplification of the more complete FKsmargon script
% It tries to do away with all fancy functions, and should work on
% rather basic infrastructure.
%
% 22.2.2018 Wayne Glettig


%% MOTOR Coordinates
% q1 = 1e-3;
% q2 = 2e-3;
% q3 = 2e-3;
% q4 = 2e-3;
% q5 = 5e-3;
% q6 = 0*pi/180;

SHX=27.35e-3;
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


%% Calculate theta (based on q3 & q4)
% calculate eqn (using symbolic math in IKsmargon)
%eqn = ((l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))^2 + (l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))^2)^(1/2) - l61
% get the derivative of eqn (in symbolic math diff(eqn, theta))
%eqn_diff =(2*(cos(theta)*(l71 + l73) + l72*sin(theta))*(l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta)) + 2*(sin(theta)*(l71 + l73) - l72*cos(theta))*(l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta)))/(2*((l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))^2 + (l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))^2)^(1/2))

% Use the newton method:
% Starting value theta_0:
theta = 0;
% Set loop conditions
loopcond = 1;
loopcounter =0;
maxloops = 30;
while (loopcond == 1)
    %Newton Formula, using f, and f_diff, 
    f = ((l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))^2 + (l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))^2)^(1/2) - l61;
    f_diff = (2*(cos(theta)*(l71 + l73) + l72*sin(theta))*(l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta)) + 2*(sin(theta)*(l71 + l73) - l72*cos(theta))*(l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta)))/(2*((l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))^2 + (l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))^2)^(1/2));
    theta = theta - (f/f_diff);
    % Increase loop counter, and stop looping if maxloop is reached
    loopcounter = loopcounter +1;
    if (loopcounter> maxloops)
        loopcond = 0;
        %ERROR no solution found!!!!
    end
    % Calculate residual error, and if sufficiently small stop looping
    if (abs(f/f_diff)<1e-9)
        loopcond = 0;
    end
end

theta;        

% Visualize it: 
fplot(@(theta) ((l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))^2 + (l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))^2)^(1/2) - l61, [-pi pi]);
grid on;
hold on;
title ('eqn = f(theta), eqn=0 -> solutions ');
xlabel('theta angle');
ylabel('eqn');
plot (theta, ((l32 - l34 + l41 - l51 + q3 - q4 + sin(theta)*(l71 + l73) - l72*cos(theta))^2 + (l31 + l33 - l42 + l52 - cos(theta)*(l71 + l73) - l72*sin(theta))^2)^(1/2) - l61, 'o');


%% the FK model is:

X= l01 + l23 + l32 + l41 + q3 + SHX*cos(theta) + l74*cos(theta) + l71*sin(theta) - SHY*cos(q5)*sin(theta) + SHZ*sin(q5)*sin(theta);
Y= l31*cos(q6) - SHZ*(cos(q5)*sin(q6) + cos(q6)*cos(theta)*sin(q5)) - SHY*(sin(q5)*sin(q6) - cos(q5)*cos(q6)*cos(theta)) - l42*cos(q6) + q1*cos(q6) - q2*sin(q6) + SHX*cos(q6)*sin(theta) - l71*cos(q6)*cos(theta) + l74*cos(q6)*sin(theta);
Z= SHY*(cos(q6)*sin(q5) + cos(q5)*cos(theta)*sin(q6)) + SHZ*(cos(q5)*cos(q6) - cos(theta)*sin(q5)*sin(q6)) + q2*cos(q6) + l31*sin(q6) - l42*sin(q6) + q1*sin(q6) + SHX*sin(q6)*sin(theta) - l71*cos(theta)*sin(q6) + l74*sin(q6)*sin(theta);
OMEGA= (180*q6)/pi;
CHI= (180*theta)/pi; 
PHI= (180*q5)/pi;

end

