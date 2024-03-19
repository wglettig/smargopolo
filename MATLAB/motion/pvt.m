function [c1 c2 c3 c4 ] = pvt(x0,x1,v0,v1,t0,t1)
%PVT Summary of this function goes here
%   Detailed explanation goes here

A= [t0^3, t0^2, t0, 1; ...
    t1^3, t1^2, t1, 1; ...
    3*t0^2, 2*t0, 1, 0; ...
    3*t1^2, 2*t1, 1, 0];
    
b = [x0;x1;v0;v1] ;

x= A\b;

c1=x(1);
c2=x(2);
c3=x(3);
c4=x(4);


end

