%% This Script tests the calcIK and calkFK models.
clear all;

%% MOTOR Coordinates
q1 = 1e-3;
q2 = 2e-3;
q3 = 2e-3;
q4 = 2e-3;
q5 = 5e-3;
q6 = 0*pi/180;
[X,Y,Z,OMEGA,CHI,PHI] = calcFK(q1,q2,q3,q4,q5,q6);
[q1,q2,q3,q4,q5,q6] = calcIK(X,Y,Z,OMEGA,CHI,PHI);

%% USER Coordinates
X = 188e-3;
Y = 0;
Z = 0;
OMEGA = 0*pi/180;
CHI = 0;
PHI = 0;
[q1,q2,q3,q4,q5,q6] = calcIK(X,Y,Z,OMEGA,CHI,PHI)
[X,Y,Z,OMEGA,CHI,PHI] = calcFK(q1,q2,q3,q4,q5,q6)


%% Write CATIA Law file
fid = fopen('smargon_laws.txt','wt');
fprintf(fid, '//  Law for Befehl.1 bis Befehl.6\n');
fprintf(fid, '---------------------------------\n');
fprintf(fid, '*COLUMNS = *TIME, Befehl.1, Befehl.2, Befehl.3, Befehl.4, Befehl.5, Befehl.6\n');
fprintf(fid, '*INTERPOLATION = polyline,spline,polyline,polyline,polyline,polyline\n');
fprintf(fid, '*UNIT = Deg,m,m,m,m,Deg\n');
fprintf(fid, '*YSCALE = 1,1,1,1,1,1\n');
t=2;
[b1,b2,b3,b4,b5,b6] = calcIK_CATIA(188e-3,0,0,0,0,0);
fprintf(fid, '%d\t%f\t%f\t%f\t%f\t%f\t%f\n', t,b1,b2,b3,b4,b5,b6);
t=4;
[b1,b2,b3,b4,b5,b6] = calcIK_CATIA(188e-3,10e-3,0,0,0,0);
fprintf(fid, '%d\t%f\t%f\t%f\t%f\t%f\t%f\n', t,b1,b2,b3,b4,b5,b6);
t=6;
[b1,b2,b3,b4,b5,b6] = calcIK_CATIA(188e-3,10e-3,10e-3,0,0,0);
fprintf(fid, '%d\t%f\t%f\t%f\t%f\t%f\t%f\n', t,b1,b2,b3,b4,b5,b6);
t=8;
[b1,b2,b3,b4,b5,b6] = calcIK_CATIA(188e-3,-10e-3,10e-3,0,0,0);
fprintf(fid, '%d\t%f\t%f\t%f\t%f\t%f\t%f\n', t,b1,b2,b3,b4,b5,b6);
t=10;
[b1,b2,b3,b4,b5,b6] = calcIK_CATIA(188e-3,-10e-3,-10e-3,0,0,0);
fprintf(fid, '%d\t%f\t%f\t%f\t%f\t%f\t%f\n', t,b1,b2,b3,b4,b5,b6);
t=12;
[b1,b2,b3,b4,b5,b6] = calcIK_CATIA(188e-3,10e-3,0,0,0,0);
fprintf(fid, '%d\t%f\t%f\t%f\t%f\t%f\t%f\n', t,b1,b2,b3,b4,b5,b6);
t=14;
[b1,b2,b3,b4,b5,b6] = calcIK_CATIA(188e-3,0,0,0,0,0);
fprintf(fid, '%d\t%f\t%f\t%f\t%f\t%f\t%f\n', t,b1,b2,b3,b4,b5,b6);
t=15;
[b1,b2,b3,b4,b5,b6] = calcIK_CATIA(188e-3,0,0,0,20,0);
fprintf(fid, '%d\t%f\t%f\t%f\t%f\t%f\t%f\n', t,b1,b2,b3,b4,b5,b6);
t=16;
[b1,b2,b3,b4,b5,b6] = calcIK_CATIA(188e-3,0,0,0,40,0);
fprintf(fid, '%d\t%f\t%f\t%f\t%f\t%f\t%f\n', t,b1,b2,b3,b4,b5,b6);
t=17;
[b1,b2,b3,b4,b5,b6] = calcIK_CATIA(188e-3,0,0,0,50,0);
fprintf(fid, '%d\t%f\t%f\t%f\t%f\t%f\t%f\n', t,b1,b2,b3,b4,b5,b6);
t=18;
[b1,b2,b3,b4,b5,b6] = calcIK_CATIA(188e-3,0,0,0,60,0);
fprintf(fid, '%d\t%f\t%f\t%f\t%f\t%f\t%f\n', t,b1,b2,b3,b4,b5,b6);
t=19;
[b1,b2,b3,b4,b5,b6] = calcIK_CATIA(188e-3,0,0,0,70,0);
fprintf(fid, '%d\t%f\t%f\t%f\t%f\t%f\t%f\n', t,b1,b2,b3,b4,b5,b6);
t=20;
[b1,b2,b3,b4,b5,b6] = calcIK_CATIA(188e-3,0,0,0,90,0);
fprintf(fid, '%d\t%f\t%f\t%f\t%f\t%f\t%f\n', t,b1,b2,b3,b4,b5,b6);

fclose(fid);