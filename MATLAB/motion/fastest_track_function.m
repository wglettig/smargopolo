function [ T X V A ] = fastest_track_function(x0,v0,xT,vT,vmax,amax,dmax,t0,dt)


%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Implementation of a fastest track motion algorithm.
% 23.8.2018, Wayne Glettig
% Fastest track from x0 to xT, with the following given constraints:
%   vmax: maximum speed of travel 
%   amax: maximum acceleration
%   dmax: maximum deceleration
% The algorithm also takes in account start and end velocities v0 & vT
% And plots the result in (x,t), (v,t) and (a,t) plots.


%% INPUT PARAMETERS:
% x0=0;  %Start position
% v0=-1;  %Start velocity
% 
% xT=-10; %Target position
% vT=-1;  %Target velocity
% 
% vmax=2; %Maxium allowed velocity
% amax=1;     %Maximum acceleration
% dmax=-amax; %Maximum deceleration
% 
% t0=0;     %Start time 
% dt=0.01;  %time step

%% CALCULATE PROFILE:
% First, we set the motion direction, depending on start and end positions. 
% Here we also make the algorithm immune to vmax, amax & dmax signs.
dir=sign(xT-x0);
amax = dir*abs(amax);
dmax = -dir*abs(dmax);
vmax = dir*abs(vmax);

% The motion profile is comprised of three phases:
% 1) Acceleration phase with full amax acceleration
% 2) Constant velocity phase with vmax
% 3) Deceleration phase with full dmax deceleration

% Now if the distance is too short to be able to accelerate up to vmax, 
% there is no constant velocity phase (phase 2).
% Let's calculate the displacement required to accelerate (v0->vmax) and 
% deccelerate (vmax->vT):
s_acc  = v0*(vmax-v0)/amax + (vmax-v0)^2/2/amax;
s_decc = vmax*(vT-vmax)/dmax + (vT-vmax)^2/2/dmax;

% Let's see if s_acc and s_decc fit within the required displacement xT-x0.
if (abs(xT-x0)<abs(s_acc+s_decc))
    %So, if it doesnt fit, an intermediate vlim is targeted, which is a 
    %local vmax to accelerate to, until it's time to decelerate.
    vlim = dir*sqrt((v0^2/2/amax - vT^2/2/dmax + xT - x0)/(1/2/amax - 1/2/dmax));
    %Also, the constant velocity phase is over a distance of zero.
    s_vconst = 0;
    % Recalculate the effective displacements using vlim instead of vmax
    s_acc  = v0*(vlim-v0)/amax + (vlim-v0)^2/2/amax
    s_decc = vlim*(vT-vlim)/dmax + (vT-vlim)^2/2/dmax;
else
    vlim =vmax;
    %The constant velocity distance is the remaining distance.
    s_vconst = (xT-x0)-(s_acc+s_decc);
end


% So now the distances have been established, let's look at the times.
% t0 --acceleration--> t1 --vconst--> t2 --deceleration--> t3
%         t_acc            t_vconst           t_decc
%
t_acc = (vlim-v0)/amax;
t_decc = (vT-vlim)/dmax;
t_vconst = s_vconst/vlim;

t1 = t0 + t_acc;
t2 = t1 + t_vconst;
t3 = t2 + t_decc;



%% STEP THROUGH PROFILE WITH GIVEN SAMPLE RATE:
%Vectors for results
T = t0:dt:t3;
X = zeros(length(T),1);
V = zeros(length(T),1);
A = zeros(length(T),1);

for (i = 1:length(T))
   t = T(i);
   if (t < t1) %acceleration phase
       a= amax;
       v= v0 + a*(t-t0);
       x= x0 + v0*(t-t0) + 1/2*a*(t-t0)^2;
   elseif (t < t2) %constant v phase
       a= 0;
       v= vlim;
       x= x0 + s_acc + v*(t-t1);
   elseif (t < t3) %deceleration phase
       a=dmax;
       v= vlim + a*(t-t2);
       x= x0 + s_acc + s_vconst + vlim*(t-t2) +1/2*a *(t-t2)^2;
   end
   %Write to vectors
   X(i)=x;
   V(i)=v;
   A(i)=a;
end

end

% %% PLOT CURVES:    
% subplot (3,1,1);
% plot(T, X,'+-');
% hold on;
% 
% subplot (3,1,2);
% plot(T, V,'+-');
% hold on;
% 
% subplot (3,1,3);
% plot(T, A,'+-');
% hold on;