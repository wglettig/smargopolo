% Implementation of a minimum jerk motion algorithm, based on Flash & 
% Hogan (1985)
% 23.8.2018, Wayne Glettig
% Path from (x0, y0) to (xT, yT) with the minimum jerk:
%    (x0, y0) : Start position in x-y plane
%    (xT, yT) : End position in x-y plane
% The algorithm also takes in account start and end velocities v0 & vT
% And plots the result in (x,t), (v,t) and (a,t) plots.


%% INPUT PARAMETERS:
x0=0;  %Start position in x
y0=0;  %Start position in y

xT=-10; %Target position in x
yT=-10; %Target position in y

t0=0;     %Start time 
dt=0.01;  %time step
tT=1;     %End time
%% CALCULATE PROFILE: STEP THROUGH PROFILE WITH GIVEN SAMPLE RATE:
%Vectors for results
T = t0:dt:tT;
X = zeros(length(T),1);
Y = zeros(length(T),1);

for (i = 1:length(T))
   t = T(i);
   
   x = x0 + (x0-xT)*(15*t^4-6*t^5-10*t^3);
   y = y0 + (y0-yT)*(15*t^4-6*t^5-10*t^3);
   
   %Write to vectors
   X(i)=x;
   Y(i)=y;
end


%% PLOT CURVES:    
plot (X,Y,'r+');
% subplot (3,1,1);
% plot(T, X,'r+-');
% hold on;
% ylabel('x(t)[m]');
% 
% subplot (3,1,2);
% plot(T, V,'g+-');
% hold on;
% ylabel('v(t)[m/s]');
% 
% subplot (3,1,3);
% plot(T, A,'b+-');
% hold on;
% ylabel('a(t)[m/s^2]');
% xlabel('t[s]');
