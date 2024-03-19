clear all;
%% INPUT PARAMETERS:
 xT=[0, 1, -2, 4, 5, -1];     %positions
 tT=[0, 1,  2, 3,  4, 5];     %times
%    v=[0, -1, 3/2, -1, -1, 0];  %velocities
vcurr=0;
acurr=0;

for i = 1:(numel(tT)-1)
    
    dt=0.01;

    [c1, c2, c3, c4, c5, v1, a1] = pt4(xT(i),xT(i+1),tT(i),tT(i+1), vcurr, acurr);
    vcurr=v1;
    acurr=a1;
    %% STEP THROUGH PROFILE WITH GIVEN SAMPLE RATE:
    %Vectors for results
    T = tT(i):dt:tT(i+1);
    X = zeros(length(T),1);
    V = zeros(length(T),1);
    A = zeros(length(T),1);
    J = zeros(length(T),1);

    for (ii = 1:length(T))
       t = T(ii);
       j= 24*c1*t   + 6*c2;
       a= 12*c1*t^2 + 6*c2*t   + 2*c3;
       v=  4*c1*t^3 + 3*c2*t^2 + 2*c3*t   + c4;
       x=    c1*t^4 +   c2*t^3 +   c3*t^2 + c4*t + c5;
%        j= 6*c1;
%        a= 6*c1*t + 2*c2;
%        v= 3*c1*t^2 + 2*c2*t + c3;
%        x= c1*t^3 + c2*t^2 + c3*t + c4;
       %Write to vectors
       X(ii)=x;
       V(ii)=v;
       A(ii)=a;
       J(ii)=j;
    end


    %% PLOT CURVES:    
    subplot (4,1,1);
    plot(T, X,'r-');
    hold on;
    ylabel('x(t)[m]');

    subplot (4,1,2);
    plot(T, V,'g-');
    hold on;
    ylabel('v(t)[m/s]');

    subplot (4,1,3);
    plot(T, A,'b-');
    hold on;
    ylabel('a(t)[m/s^2]');
    
    subplot (4,1,4);
    plot(T, J,'k-');
    hold on;
    ylabel('j(t)[m/s^3]');
    xlabel('t[s]');
end

    subplot (4,1,1);
    stem(tT,xT);
    hold on;
   