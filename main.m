clear; clc
% Ts = 1;
% Add yalmip
setup_paths

Ts = .4; % Time step size (in seconds)
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
xylim = 1*10^4;
params.xylim = xylim;
psi1 = -pi/2; xy = [-1E4; 3000]; v = 200;
psi2 = -pi/2;
psi3 = pi/2;
psi4 = pi/4;
psi5 = 0;
psi6 = 0;
psi7 = -pi/2;
x0a = [0;       4000;   v*cos(psi1);v*sin(psi1)];
x0b = [-8000;   4000;   v*cos(psi2);v*sin(psi2)];
x0c = [-9000;   0;      v*cos(psi3);v*sin(psi3)];
x0d = [-1500;   -7000;      v*cos(psi4);v*sin(psi4)];
x0e = [-8000;   -4000;  v*cos(psi5);v*sin(psi5)];
x0f = [-4000;   -4000;  v*cos(psi6);v*sin(psi6)];
x0g = [4000;    4000;   v*cos(psi7);v*sin(psi7)];
a = linearizedPlane('1',x0a,psi1,v, xylim, Ng);
b = linearizedPlane('2',x0b,psi2,v,xylim, Ng);
c = linearizedPlane('3',x0c,psi3,v,xylim, Ng);
d = linearizedPlane('4',x0d,psi4,v,xylim, Ng);
e = linearizedPlane('5',x0e,psi5,v,xylim, Ng);
f = linearizedPlane('6',x0f,psi6,v,xylim, Ng);
g = linearizedPlane('7',x0g,psi6,v,xylim, Ng);
% populates relevant fields of params
params = addPlane(a,params, N);
params = addPlane(b,params, N);
params = addPlane(c,params, N);
params = addPlane(d,params, N);
params = addPlane(e,params, N);
params = addPlane(f,params, N);
% params = addPlane(g,params, N);

% params.costs('4', '4') = d.constant_radius_cost(5000);

order = {};
% landing_id = '4';
order = { '1','6', '4','5', '2',  '3'};
% order = {'1','2','3'};
% 
landing_id = order{1}; % choose which plane we want to land
order = order(2:end);
savePlot = 1;
dir1 = strcat(string(date)," ", string(hour(datetime)),".",string(minute(datetime)),"fig10");
dir2 = strcat(string(date)," ", string(hour(datetime)),".",string(minute(datetime)),"fig1");
if savePlot
    mkdir(strcat('plots/',dir1));
    mkdir(strcat('plots/',dir2));
end
params.lzdia = 1000; %landing zone diameter
v = VideoWriter('Plane.avi','Motion JPEG AVI');
set(v,'FrameRate', 5)
open(v);
for j = 1:Ng %global simulation loop
    fprintf("Global timestep: %d\n",j);
    Ts = max(set_ts(params, N, landing_id), 0.8);
    atcMPC(params,Ts,N); %mpc controller call
    for plane = values(params.aircraft_list)
        plane{1}.recordAndAdvanceState(); %applying control input to each plane
    end
    frame(j) = plotPos2(params,j); %update on plot
    
    if (dist_center(params, landing_id) < params.lzdia)
        removePlane(params, landing_id)
        numkeys = keys(params.aircraft_list);
        if numel(numkeys) == 0
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
        saveas(figure(10), strcat('plots/',dir1,'/',num2str(j), '.jpg'));
        saveas(figure(1), strcat('plots/',dir2,'/',num2str(j), '.jpg'));
    end
end
writeVideo(v,frame)
close(v);
% plotStateAndInputs(params);
disp('done')