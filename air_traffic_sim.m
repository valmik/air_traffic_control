clear all; clc; disp('started');
% Setup
tic
% 
Ts = .5;
xy = [-1E3; 300];
v = 200;
psi = 0;
x0 = [100;xy(2);v*cos(psi);v*sin(psi)];
x0b = [xy(1);700;v*cos(psi);v*sin(psi)];
x0c = [-800;-xy(2);v*cos(psi);v*sin(psi)];
x0d = [-700;-xy(2);v*cos(psi);v*sin(psi)];
x0e = [-600;-xy(2);v*cos(psi);v*sin(psi)];
a = linearizedPlane('1',x0,psi,v,Ts);
b = linearizedPlane('2',x0b,psi,v,Ts);
c = linearizedPlane('3',x0c,psi,v,Ts);
d = linearizedPlane('4',x0d,psi,v,Ts);
e = linearizedPlane('5',x0e,psi,v,Ts);
numPlanes = 20;
aircraft_list = [a; b; c; d; e];
for i = 1:numPlanes
    x0e = [-rand*1000;rand*2000-1000;v*cos(psi);v*sin(psi)];
    aircraft_list = [aircraft_list; linearizedPlane('5',x0e,psi,v,Ts)];
end
% x0a = [-1E3;300;200;0;10^9];
% x0b = [-1E3;-300;200;0;10^9];
% Ts = .1;
% a = PlaneModel('1',x0a);
% b = PlaneModel('1',x0b);

N = 20; %sim horizon
% timesteps = sdpvar(1, N); %length of each timestep
timesteps = Ts*ones(1,N);
for i = 1:numel(aircraft_list) %setup aircraft variables
   aircraft_list(i) = aircraft_list(i).setup_yalmip(N, timesteps); 
end
constraints = [];
% for k = 1:N
%     constraints = [constraints, (0.01 <= timesteps(k) <= 0.5):'Timestep constraint'];
% end
% Define Cost
basic_cost = 0;
for i = 1:numel(aircraft_list)
    basic_cost = basic_cost + aircraft_list(i).yalmip_cost;
end
disp('collision');
collision_cost = 0;
if numel(aircraft_list) >= 2
    for i = 1:numel(aircraft_list)
        for j = (i+1):numel(aircraft_list)
            fprintf("i: %d j:%d \n",[i j]);
            for k = 1:N+1
                vector_diff = aircraft_list(i).x_yalmip(:,k) - aircraft_list(j).x_yalmip(:,k);
                radius = max(aircraft_list(i).radius^2, aircraft_list(j).radius^2);
                collision_cost = collision_cost - (vector_diff(1)^2 + vector_diff(2)^2 - radius^2);
            end
        end
    end
end
cost = basic_cost + .01*collision_cost;

% Constraints
% individual constraints
for i = 1:numel(aircraft_list)
   constraints = [constraints, ...
       aircraft_list(i).yalmip_constraints];
end

% collision constraints
% if numel(aircraft_list) >= 2
%     for i = 1:numel(aircraft_list)
%         for j = (i+1):numel(aircraft_list)
%             for k = 1:N+1
%                 vector_diff = aircraft_list(i).x_yalmip(:,k) - aircraft_list(j).x_yalmip(:,k);
%                 radius = max(aircraft_list(i).radius.^2, aircraft_list(j).radius.^2);
%                 constraints = [constraints, ... 
%                     ((vector_diff(1)^2 + vector_diff(2)^2) >= radius ...
%                     ):[aircraft_list(i).id, ' ', aircraft_list(j).id, ' collision constraint']];
%             end
%         end
%     end
% end

% Final Constraint
i=1;
% constraints = [constraints, ...
%     (aircraft_list(i).x_yalmip(1, N+1) == 0):['Final Constraint'], ...
%     (aircraft_list(i).x_yalmip(2, N+1) == 0):['Final Constraint']];

% Solve
options = sdpsettings('solver', 'IPOPT','verbose',3, 'showprogress', 5);
            
options.ipopt.mu_strategy      = 'adaptive';
options.ipopt.max_iter         = 1e4;
options.ipopt.tol              = 1e-3;
options.ipopt.linear_solver    = 'ma57';
options.ipopt.ma57_automatic_scaling = 'yes';
options.ipopt.linear_scaling_on_demand = 'yes';
options.ipopt.hessian_approximation = 'limited-memory';
options.ipopt.limited_memory_update_type = 'bfgs';  % {bfgs}, sr1
options.ipopt.limited_memory_max_history = 10;  % {6}
options.ipopt.max_cpu_time = 1e8;
disp('opt');
toc
tic
    exitval_opt = optimize(constraints, cost, options)
toc

% Plot
figure(1);clf
hold on
legend_str = cell(size(aircraft_list));
for i = 1:numel(aircraft_list)
    x_opt = double(aircraft_list(i).x_yalmip);
    plot(x_opt(1, :), x_opt(2, :), '--*')
    hold on
    legend_str{i} = aircraft_list(i).id;
end
title('Position plot')
xlim(1.2E3*[-1 1]);
ylim(1.2E3*[-1 1]);
% legend(legend_str)
grid

% States
figure(2); clf;
hold on
legend_str = cell(size(aircraft_list));
nx = aircraft_list(1).nx;
nu = aircraft_list(1).nu;

for i = 1:numel(aircraft_list)
    x_opt = double(aircraft_list(i).x_yalmip);
    for j = 1:nx
        subplot(nx,1,j);
        hold on
        plot(x_opt(j, :), '--*')
    end
    
    legend_str{i} = aircraft_list(i).id;
end
subplot(nx, 1, 1)
hold on
title('State plot')

legend(legend_str)

figure(3); clf; %inputs
for i = 1:numel(aircraft_list)
    u_opt = double(aircraft_list(i).u_yalmip);
    for j = 1:nu
    subplot(nu,1,j);
    hold on
    plot(u_opt(j, :), '--*')
    end
    legend_str{i} = aircraft_list(i).id;
end
subplot(nu, 1, 1);
hold on
title('bank angle')
legend(legend_str)
subplot(nu,1,2);
title('thrust');


% Test collision constraints:

if numel(aircraft_list) >= 2
    for i = 1:numel(aircraft_list)
        for j = (i+1):numel(aircraft_list)
            for k = 1:N+1
                vector_diff = double(aircraft_list(i).x_yalmip(:,k)) - double(aircraft_list(j).x_yalmip(:,k));
                radius = max(aircraft_list(i).radius.^2, aircraft_list(j).radius.^2);
                if (vector_diff(1)^2 + vector_diff(2)^2) < radius
                    disp('FAIL collision constr')
                    break;
                end
            end
        end
    end
end

% Test collision cost
for i = 1:numel(aircraft_list)
    for j = (i+1):numel(aircraft_list)
        for k = 1:N+1
            vector_diff = double(aircraft_list(i).x_yalmip(:,k)) - double(aircraft_list(j).x_yalmip(:,k));
            radius = max(aircraft_list(i).radius^2, aircraft_list(j).radius^2);
            log((vector_diff(1)^2 + vector_diff(2)^2)/radius);
        end
    end
end
figure(1);