



% List of Points at time (one point = one line in matrix)
% Each Line: t, x, y, z
P = [0, 0,0,0;...
     1, 1,1,1;...
     2, 0,1,1;...
     3, 2,3,1];

% For each dimension:
     
% subplot(1,2,1);
% plot(P(:,2), P(:,3));
% xlabel('x');
% ylabel('y');
% subplot(1,2,2);
% plot(P(:,1), P(:,2));
% xlabel('t');
% ylabel('x');



%Points(t, x,y,z) = pt(P)
%Look at each dimension separately
%Boundary conditions:
  % maxaccel = 0.1

% From the PMAC Documentation for PVT Motion profiles:
% From the specified parameters for the move piece, and the beginning 
% position and velocity (from the end of the previous piece), PMAC 
% computes the only third-order position trajectory path to meet the 
% constraints. This results in linearly changing acceleration, a parabolic 
% velocity profile, and a cubic position profile for the piece.

% Command:
% PVT300
% X5:50
% Axis X, t=300, p=5, v=50
clf
%Vectors of results
T=[];
X=[];
V=[];
A=[];
%Input Parameters
x0=0;
v0=0;
a0=0;

xT=7;


vmax=1;
amax=1;
dt=0.001;
errorband=0.0001;

%Start values
x=x0;
v=v0;
a=a0;


t=0;
loop =1;
asign = 1;
while (loop)
    t=t+dt; 
    %bremsweg s = v0^2/2/a
    %if (xT-x < v^2/2/amax) %check bremsweg. if smaller, decel full.
    xrem = xT-x;  %remaining x  
    %if (abs(xrem) < abs(v*dt - 0.5*amax*dt^2))%check bremsweg. if smaller, decel full.
    if (abs(xT-x) < abs(v^2/2/amax))%check bremsweg. if smaller, decel full.
        %a=-sign(xT-x)*amax;
        asign =-1;
        %disp(['deccelerating t=', num2str(t),' a=',num2str(a)])
    else 
        asign = 1;
    end
    
    a=sign(xT-x)*asign*amax;  %calculate new a
    v=v+a*dt;                 %calculate new v
    if abs(v)>vmax %limit velocity
        v=sign(v)*vmax;
        a=0;
    end
    x=x + v*dt + 0.5*a*dt^2;  %calculate new x
    if (abs(xT-x)<errorband || t > 10) %if reached target or loopcounter = 1000
        loop=0;
    end
%    if (abs(xT-x) > abs(xT-x0))
%        disp('target couldnt be reached');
%        loop =0;
%    end
    T=[T,t];
    X=[X,x];
    V=[V,v];
    A=[A,a];
end
subplot (3,1,1);
plot(T, X,'r+');
ylabel('x(t)[m]');

subplot (3,1,2);
plot(T, V,'g+');
ylabel('v(t)[m/s]');

subplot (3,1,3);
plot(T, A,'b+');
ylabel('a(t)[m/s^2]');
xlabel('t[s]');

hold on;