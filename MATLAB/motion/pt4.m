function [c1 c2 c3 c4 c5 v1 a1] = pt(x0,x1,t0,t1, v0,a0)

%PVT Summary of this function goes here
%   Detailed explanation goes here

A= [   t0^4,   t0^3,   t0^2, t0, 1,0,0; ...
       t1^4,   t1^3,   t1^2, t1, 1,0,0; ...
     4*t0^3, 3*t0^2, 2*t0  ,  1, 0,0,0; ...
     4*t1^3, 3*t1^2, 2*t1  ,  1, 0,-1,0; ...
    12*t0^2, 6*t0  ,   2   ,  0, 0,0,0 ; ...
    12*t1^2, 6*t1  ,   2   ,  0, 0,0,-1];
    
b = [x0;x1;v0;0;a0;0] ;

x= A\b;

c1=x(1);
c2=x(2);
c3=x(3);
c4=x(4);
c5=x(5);
v1=x(6);
a1=x(7);


end

% 
% %PVT Summary of this function goes here
% %   Detailed explanation goes here
% 
% A= [t0^3, t0^2, t0,  1,0,0; ...
%     t1^3, t1^2, t1,  1,0,0; ...
%     3*t0^2, 2*t0, 1, 0,0,0; ...
%     3*t1^2, 2*t1, 1, 0,-1,0; ...
%     6*t0, 2,  0,   0,0,0; ...
%     6*t1  , 2,  0,   0,0,-1];
%     
% b = [x0;x1;v0;0;a0;0] ;
% 
% x= A\b;
% 
% c1=x(1);
% c2=x(2);
% c3=x(3);
% c4=x(4);
% v1=x(5);
% a1=x(6);
% 
% 
% end
% 
