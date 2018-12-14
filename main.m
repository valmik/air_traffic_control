clear; clc
% Ts = 1;
% Add yalmip
setup_paths

Ts = .3; % Time step size (in seconds)
N = 10; %MPC simulation horizon
Ng = 1000;%global horizon

params = struct();
%params struct holds constraints and costs that are shared between runs,
% to avoid the overhead of regenerating these per run (collision costs
% scale poorly with number of planes
% constraints: none currently. dynamics constraints are not shared between
% mpc runs since timesteps change. dynamics constraints are added in atcMPC
% costs: collision costs & distance from origin cost
params.aircraft_list = containers.Map;
params.color_list = containers.Map;
params.costs = MapNested();
params.collision_constraints = MapNested;
params.Ng = Ng; 

psi1 = -pi/2; xy = [-1E3; 300]; v = 200;
psi2 = pi/2;
psi3 = pi/3;
psi4 = pi/2;
psi5 = -pi/2;
x0a = [-8000; 3000;v*cos(psi1);v*sin(psi1)];
x0b = [-8000; -3000;v*cos(psi2);v*sin(psi2)];
x0c = [-3000; -1000;v*cos(psi3);v*sin(psi3)];
x0d = [2000; -3000;v*cos(psi2);v*sin(psi2)];
x0e = [-3000; -8000;v*cos(psi2);v*sin(psi2)];
x0f = [+3000; -8000;v*cos(psi5);v*sin(psi5)];
a = linearizedPlane('1',x0a,psi1,v,Ng);
b = linearizedPlane('2',x0b,-pi/4,v,Ng);
c = linearizedPlane('3',x0c,psi2,v,Ng);
d = linearizedPlane('4',x0d,psi2,v,Ng);
e = linearizedPlane('5',x0e,psi2,v,Ng);
f = linearizedPlane('6',x0f,psi5,v,Ng);

% populates relevant fields of params
params = addPlane(a,params, N);
params = addPlane(b,params, N);
% params = addPlane(c,params, N);
% params = addPlane(e,params, N);
% params = addPlane(f,params, N);
% params = addPlane(d,params, N);

% params.costs('4', '4') = d.constant_radius_cost(5000);

order = {};
% landing_id = '4';
% order = { '3', '2', '1','5', '6'};%, '4'};
order = {'1','2'};
% 
landing_id = order{1}; % choose which plane we want to land
order = order(2:end);
savePlot = 1;
dir = strcat(string(date)," ", string(hour(datetime)),".",string(minute(datetime)));
if savePlot
    mkdir(strcat('plots/',dir));
end
params.lzdia = 1000; %landing zone diameter
for j = 1:Ng %global simulation loop
    fprintf("Global timestep: %d\n",j);
    Ts = max(set_ts(params, N, landing_id), 0.5);
    atcMPC(params,Ts,N); %mpc controller call
    for plane = values(params.aircraft_list)
        plane{1}.recordAndAdvanceState(); %applying control input to each plane
    end
    plotPos(params); %update on plot
    
    if (dist_center(params, landing_id) < params.lzdia)
        removePlane(params, landing_id)
        keys = keys(params.aircraft_list);
        if numel(keys) == 0
            break
        else
%             keys = keys(params.aircraft_list);
%             landing_id = keys{1};
            landing_id = order{1};
            if numel(order) > 1
                order = order(2:end);
            end
        end
    end
    if savePlot
        saveas(figure(10), strcat('plots/',dir,'/',num2str(j), '.jpg'));
    end
end

% plotStateAndInputs(params);
disp('done')