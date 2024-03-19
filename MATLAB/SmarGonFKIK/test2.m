clear all;
syms offsetX offsetY offsetZ ...
     alpha beta gamma;

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


l31 = 10e-3;
l32 = 20e-3;
l33 = 10e-3;
l34 = 20e-3;
l41 = 40e-3;
l42 = 30e-3;
l51 = 50e-3;
l52 = 50e-3;
l61 = 50e-3;
l71 = 10e-3;
l72 = 20e-3;
l73 = 10e-3;
l74 = 20e-3;
q3=0;
q4=0;


syms x y z theta
H43 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [l32+q3,l31,0,0,0,0]);
H53 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [l34+q4,-l33,0,0,0,0]);
H35 = inv(H53)
H75 = H35*H43*H74

P770in7 = [-l72;-l71-l73;0;1]


f=H75*P770in7-[x;y;z;1]
f(4)=(x + l51)^2 + (y + l52)^2 + z^2 - l61^2 

 
% Jacobian of FK f function derived over the searched variables
J= jacobian(f,[x,y,z,theta]);

Jinv = pinv(J);
tic
% Algorithm for IK (q are motor coords, x are user coords)
xt=[0;0;0;0]; %SET TARGET US ER COORDS 
q0=[-l51+l61;-l52;0;0]; %motor start values
qc = q0; %set current q values for loop
loopcond = 1;
loopcounter=0;
while loopcond
    xc = vpa(subs(f, [x,y,z,theta], qc')); %get current x values based on q 
    deltax=xt-xc %get x error (target - current)
    if (abs(deltax)<1e-9) | (loopcounter > 4)%if abs error small enough, get out of loop
        loopcond=0;
    end
    Jinvc=vpa(subs(Jinv, [x,y,z,theta], qc')); %inv Jacobian with current q
    deltaq=Jinvc*deltax; %By multiplying the x error with Jinv, a q correction can be deduced
    qc = qc+deltaq;%update current motor values
    loopcounter=loopcounter+1;   
end
q = qc %output q as the motor coordinates
toc





