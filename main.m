clear; clc
Ts = 1;
N = 20; %MPC simulation horizon
Ng = 100;%global horizon
params = struct();
params.constr = [];
params.obj = 0;
params.aircraft_list = [];
params.Ng = Ng; 

psi1 = 0; xy = [-1E3; 300]; v = 200;
psi2 = 0;
psi3 = pi/3;
x0a = [-6000; 0;v*cos(psi1);v*sin(psi1)];
x0b = [-4500; 0;60*cos(psi2);60*sin(psi2)];
x0c = [-3000; -1000;v*cos(psi3);v*sin(psi3)];
x0d = [2000; -3000;v*cos(psi2);v*sin(psi2)];
a = linearizedPlane('1',x0a,psi1,v,Ng);
b = linearizedPlane('2',x0b,psi2,v,Ng);
c = linearizedPlane('2',x0c,psi2,v,Ng);
d = linearizedPlane('2',x0d,psi2,v,Ng);
params = addPlane(a,params, N);
params = addPlane(b,params, N);
% params = addPlane(c,params, N);
% params = addPlane(d,params, N);

for j = 1:Ng
    disp(j);
    atcMPC(params,Ts,N)
    for ii = 1:numel(params.aircraft_list)
        plane = params.aircraft_list(ii);
        plane.recordAndAdvanceState();
    end
    plotPos(params)
end

% plotPos(params)
