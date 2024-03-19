clear all;
syms offsetX offsetY offsetZ ...
     alpha beta gamma...
     q1 q2 q3 q4 q5 q6;

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
l61 = 50e-3;
l71 = 10e-3;
l72 = 20e-3;
l73 = 10e-3;
l74 = 20e-3;

syms theta  tmp1 tmp2 tmp3 tmp4 tmp5
%H73(via Part 4) = H73 (via Parts 5 & 6)
%H73(via Part 4):
H43 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [q3+l32,l31,0,0,0,0]); %Slider #3
H74 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [l41,-l42,0,0,0,theta]); %theta angle
H73a = H43*H74

%(only thing unknown is theta)

%H73 (via Parts 5 & 6)
%H73 = H53*H65*H76
H53 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [q4+l34,-l33,0, 0,0,0]); %Slider #4 + Ball joint
H65 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [0,0,l61,tmp1,tmp2,tmp3]); %long joint
H7a6 = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
               [0,0,0,tmp4,tmp5,0]); %ball joint
H77a = subs(HBA,[offsetX,offsetY,offsetZ,alpha,beta,gamma],...
                [l72,-l71-l72,0,0,0,0]);
H73b = H53*H65*H7a6*H77a
 %(unknowns tmp1 tmp2 tmp3 tmp4 tmp5)
 
f= H73a(1:3,4)-H73b(1:3,4)
f1= H73a(1:3,1)-H73b(1:3,1)
f2= H73a(1:3,2)-H73b(1:3,2)
f3= H73a(1:3,3)-H73b(1:3,3)
 
% Jacobian of FK f function derived over motor coordinates
J= jacobian(f,[theta,tmp1,tmp2,tmp3,tmp4,tmp5]);
J1= jacobian(f1,[theta,tmp1,tmp2,tmp3,tmp4,tmp5]);
J2= jacobian(f2,[theta,tmp1,tmp2,tmp3,tmp4,tmp5]);
J3= jacobian(f3,[theta,tmp1,tmp2,tmp3,tmp4,tmp5]);

J= [J;J1;J2;J3]
Jinv = pinv(J);
 v   tic
% Algorithm for IK (q are motor coords, x are user coords)
xt=[2;2;2;30]; %SET TARGET US ER COORDS 
q0=[0;0;0;0;0]; %motor start values
qc = q0; %set current q values for loop
loopcond = 1;
loopcounter=0;
while loopcond
    xc = vpa(subs(f, [q1,q2,q3,q4,q5], qc')); %get current x values based on q 
    deltax=xt-xc; %get x error (target - current)
    if (abs(deltax)<1e-9) | (loopcounter > 4)%if abs error small enough, get out of loop
        loopcond=0;
    end
    Jinvc=vpa(subs(Jinv, [q1,q2,q3,q4,q5], qc')); %inv Jacobian with current q
    deltaq=Jinvc*deltax; %By multiplying the x error with Jinv, a q correction can be deduced
    qc = qc+deltaq;%update current motor values
    loopcounter=loopcounter+1;   
end
q = qc %output q as the motor coordinates
toc


