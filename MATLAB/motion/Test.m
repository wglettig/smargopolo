% [ T X V A ] = fastest_track_function(x0,v0,xT,vT,vmax,amax,dmax,t0,dt)


% List of Points at time (one point = one line in matrix)
% Each Line: t, x, y, z
P = [0, 0,0,0;...
     1, 1,1,1;...
     2, 0,1,1;...
     3, 2,3,1;...
     4, 4,4,3];
 
% For each dimension interpolate the fastest track function
X = pvt_varVmax(P(:,1), P(:,2));
plot (X,Y,Z);

 