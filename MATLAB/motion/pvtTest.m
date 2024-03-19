clear all;
%% INPUT PARAMETERS:
% tT=[0, 1,  2, 3,  4, 5];     %times
% xT=[0, 1, -2, 4, 5, -1];     %positions
% vT=[0, 0,  0, 0, 0, 0 ];     %velocities
n=10
tT=linspace(0,10,n);
xT=rand(n,1);
vT=zeros(n,1);
for i = 2:(numel(tT)-1)
   vT(i)=(xT(i+1)-xT(i-1))/tT(2)

end


dt=0.01;

for i = 1:(numel(tT)-1)
    [c1, c2, c3, c4 ] = pvt(xT(i),xT(i+1), vT(i),vT(i+1), tT(i),tT(i+1));

    %% STEP THROUGH PROFILE WITH GIVEN SAMPLE RATE:
    %Vectors for results
    T = tT(i):dt:tT(i+1);
    X = zeros(length(T),1);
    V = zeros(length(T),1);
    A = zeros(length(T),1);
    J = zeros(length(T),1);

    for (ii = 1:length(T))
       t = T(ii);
       j= 6*c1;
       a= 6*c1*t + 2*c2;
       v= 3*c1*t^2 + 2*c2*t + c3;
       x= c1*t^3 + c2*t^2 + c3*t + c4;
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
   