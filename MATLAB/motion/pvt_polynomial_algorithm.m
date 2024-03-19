% Implementation of a PVT motion algorithm.
% 23.8.2018, Wayne Glettig
% Fastest track from x0 to xT, with the following given constraints:
%   vmax: maximum speed of travel 
%   amax: maximum acceleration
%   dmax: maximum deceleration
% The algorithm also takes in account start and end velocities v0 & vT
% And plots the result in (x,t), (v,t) and (a,t) plots.


%% INPUT PARAMETERS:
x0=0;  %Start position
v0=1;  %Start velocity

xT=-5; %Target position
vT=0;  %Target velocity

vmax=0.5; %Maxium allowed velocity
amax=0.4;     %Maximum acceleration
dmax=-amax; %Maximum deceleration

t0=0;     %Start time 
tT=20;     %End Time
dt=0.01;  %time step

%% CALCULATE PROFILE:


A= [t0^3, t0^2, t0, 1; ...
    tT^3, tT^2, tT, 1; ...
    3*t0^2, 2*t0, 1, 0; ...
    3*tT^2, 2*tT, 1, 0];
b= [x0;xT;v0;vT] ;

x= A\b;

c1=x(1);
c2=x(2);
c3=x(3);
c4=x(4);




%% STEP THROUGH PROFILE WITH GIVEN SAMPLE RATE:
%Vectors for results
T = t0:dt:tT;
X = zeros(length(T),1);
V = zeros(length(T),1);
A = zeros(length(T),1);
J = zeros(length(T),1);

for (i = 1:length(T))
   t = T(i);
   a= 6*c1*t + 2*c2;
   v= 3*c1*t^2 + 2*c2*t + c3;
   x= c1*t^3 + c2*t^2 + c3*t + c4;
   %Write to vectors
   X(i)=x;
   V(i)=v;
   A(i)=a;
end


%% PLOT CURVES:    
subplot (3,1,1);
plot(T, X,'r+-');
hold on;
ylabel('x(t)[m]');

subplot (3,1,2);
plot(T, V,'g+-');
hold on;
ylabel('v(t)[m/s]');

subplot (3,1,3);
plot(T, A,'b+-');
hold on;
ylabel('a(t)[m/s^2]');
xlabel('t[s]');
