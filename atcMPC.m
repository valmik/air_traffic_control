function out = atcMPC(params,Ts,N)
% tic
timesteps = Ts*ones(1,N);
constr2  = [];
% dynamics constraints
for i = 1:numel(params.aircraft_list)
   plane = params.aircraft_list(i);
   constr2 = [constr2 plane.x_yalmip(:,1) == plane.state];
   plane.set_yalmip_constraints(timesteps);
   constr2 = [constr2 plane.yalmip_constraints];
end

constr = [params.constr constr2];
% constr = constr2;
% Final Constraint
% i=5;
% constraints = [constraints, ...
%     (aircraft_list(i).x_yalmip(1, N+1) == 0):['Final Constraint'], ...
%     (aircraft_list(i).x_yalmip(2, N+1) == 0):['Final Constraint']];

% Solve
opts = setOptOptions();
% toc
% disp('start optimization');
% tic
fprintf('opt start\n');
exitval_opt = optimize(constr, params.obj, opts);
fprintf('opt end\n');
% toc
% value(params.obj)
if exitval_opt.problem == 1
    fprintf("infeasible \n");
elseif exitval_opt.problem ~= 0
    fprintf('error\n');
end
out = exitval_opt.problem;

end