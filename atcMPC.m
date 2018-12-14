function out = atcMPC(params,Ts,N)
% tic
timesteps = Ts*ones(1,N);
constr2  = [];

% Construct Constraints
for planecell = values(params.aircraft_list)
   plane = planecell{1};
%    plane.state
   constr2 = [constr2 (plane.x_yalmip(:,1) == plane.state):[plane.id, ' initial state']];
   plane.set_yalmip_constraints(timesteps);
   constr2 = [constr2 plane.yalmip_constraints];
end

% add collision constraints
coll_con = [];
for map = values(params.collision_constraints)
    for val = values(map{1})
        coll_con = [coll_con, val{1}];
    end
end

constr = [coll_con constr2];

% Construct Cost
cost = 0;
for map = values(params.costs)
    for val = values(map{1})
        cost = cost + val{1};
    end
end

% Solve
opts = setOptOptions();
% toc
% disp('start optimization');
% tic
fprintf('opt start\n');
exitval_opt = optimize(constr, cost, opts);
fprintf('opt end\n');
% toc
% value(params.obj)
if exitval_opt.problem == 1
    fprintf("infeasible \n");
elseif exitval_opt.problem ~= 0
    fprintf('error\n');
% toc
end
out = exitval_opt.problem;

end