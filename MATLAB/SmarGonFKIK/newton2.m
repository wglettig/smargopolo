function [x , res , xvec , resvec] = newton( f, df, x0, maxiter, tol )
%
% NEWTON Newton's Method
%   Newton's method for finding better approximations to the 
%   zeroes of a real-valued function.
%
% Input:
%   f - input funtion
%   df - derived input function
%   x0 - initial aproximation
%   maxiter - maximum number of iterations allowed
%   tol - the required tolerance
%
% Output:
%   x - variable containing approximation to x(*)
%   res- variable containing the residual |f(x)| evaluated at the approximate solution x
%   xvec- vector containing {x(k)}, including initial guess and final approximation
%   resvec- vector of the same dimension of xvec, containing {|f(xk)|}.
%
function x(k) = newton( f, df, x0, maxiter, tol )
k(1:maxiter)
f = @x(k)
df = @(x(k))'
x(k)= x0-(f(x0)/df(x0))
x(k+1) = x(k) - (f(x(k))/df(x(k)))
end
%continue iterating until stopping conditions met
%output depends on success of algorithm
while (iter < maxiter)
	while (abs(f(n)-(f(n-1)) > tol)
		x(1)= x0-(f(x0)/df(x0))
	elseif (iter > maxiter) 
		error('maximum amount of iterations exceeded')
	else error('distance between last 2 points is less than stated tolerance')
	end
end
fprintf('step(k)      x(k)       |f(xk)|     |x(k+1)-x(k)|')
fprintf('------    -----------  ---------    -------------\n')
fprintf('%d %d %.4f %.4f\n', k, x(k), mod(f(xk)), mod(x(k+1)?x(k)) )