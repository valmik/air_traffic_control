clear; clc
Ts = .5;
N = 10; %MPC simulation horizon
params = struct();
params.constr = [];
params.obj = 0;
params.aircraft_list = [];
psi = 0; xy = [-1E3; 300]; v = 200;
x0a = [100;xy(2);v*cos(psi);v*sin(psi)];
x0b = [xy(1);700;v*cos(psi);v*sin(psi)];
a = linearizedPlane('1',x0a,psi,v);
b = linearizedPlane('2',x0b,psi,v);
params = addPlane(a,params, N);
params = addPlane(b,params, N);

atcMPC(params,Ts,N)
plotPos(params)