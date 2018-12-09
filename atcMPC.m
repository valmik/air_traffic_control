function out = atcMPC(params,Ts,N)
tic
timesteps = Ts*ones(1,N);
% dynamics constraints
for i = 1:numel(params.aircraft_list)
   params.aircraft_list(i).set_yalmip_constraints(timesteps);
   params.constr = [params.constr, params.aircraft_list(i).yalmip_constraints];
end


% Final Constraint
% i=5;
% constraints = [constraints, ...
%     (aircraft_list(i).x_yalmip(1, N+1) == 0):['Final Constraint'], ...
%     (aircraft_list(i).x_yalmip(2, N+1) == 0):['Final Constraint']];

% Solve
opts = setOptOptions();
toc
disp('start optimization');
tic
    exitval_opt = optimize(params.constr, params.obj, opts)
toc