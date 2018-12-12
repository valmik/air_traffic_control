clear; clc

% Add yalmip
setup_paths

Ts = 2; % Time step size (in seconds)
N = 20; %MPC simulation horizon
Ng = 20;%global horizon
params = struct();
%params struct holds constraints and costs that are shared between runs,
% to avoid the overhead of regenerating these per run (collision costs
% scale poorly with number of planes
% constraints: none currently. dynamics constraints are not shared between
% mpc runs since timesteps change. dynamics constraints are added in atcMPC
params.constr = [];
% costs: collision costs & distance from origin cost
params.obj = 0;
params.aircraft_list = [];
params.Ng = Ng; 

psi1 = 0; xy = [-1E3; 300]; v = 200;
psi2 = 0;
psi3 = pi/3;
x0a = [-4500; 3000;v*cos(psi1);v*sin(psi1)];
x0b = [-6000; -3000;v*cos(psi2);v*sin(psi2)];
x0c = [-3000; -1000;v*cos(psi3);v*sin(psi3)];
x0d = [2000; -3000;v*cos(psi2);v*sin(psi2)];
a = linearizedPlane('1',x0a,psi1,v,Ng);
b = linearizedPlane('2',x0b,psi2,v,Ng);
c = linearizedPlane('3',x0c,psi2,v,Ng);
d = linearizedPlane('4',x0d,psi2,v,Ng);
% populates relevant fields of params
params = addPlane(a,params, N);
% params = addPlane(b,params, N);
% params = addPlane(c,params, N);
% params = addPlane(d,params, N);

landing_index = 1; % choose which plane we want to land

for j = 1:Ng %global simulation loop
    disp(j);
    
    Ts = set_ts(params, N, landing_index)
%     Ts = 0.9544;

    if atcMPC(params,Ts,N) %mpc controller call
        break; % break if there's an error
    end
    for ii = 1:numel(params.aircraft_list) 
        plane = params.aircraft_list(ii);
        plane.recordAndAdvanceState(); %applying control input to each plane
    end
    plotPos(params) %update on plot
    
    if dist_center(params, landing_index) < 500
        break
    end
    pause
end

plotStateAndInputs(params);
disp('done')