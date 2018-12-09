clear; clc
Ts = .5;
N = 10; %MPC simulation horizon
Ng = 10;%global horizon
params = struct();
params.constr = [];
params.obj = 0;
params.aircraft_list = [];
params.Ng = Ng; 

psi = 0; xy = [-1E3; 300]; v = 200;
x0a = [100;xy(2);v*cos(psi);v*sin(psi)];
x0b = [xy(1);700;v*cos(psi);v*sin(psi)];
a = linearizedPlane('1',x0a,psi,v,Ng);
b = linearizedPlane('2',x0b,psi,v,Ng);
params = addPlane(a,params, N);
params = addPlane(b,params, N);

for j = 1:Ng
    disp(j);
    atcMPC(params,Ts,N)
    for ii = 1:numel(params.aircraft_list)
        plane = params.aircraft_list(ii);
        plane.recordAndAdvanceState();
    end
end

plotPos(params)
