function options = setOptOptions()

% options = sdpsettings('solver', 'IPOPT','verbose',1, 'showprogress', 1, 'debug', 1);

options = sdpsettings('solver', 'IPOPT','verbose', false, 'showprogress', false, 'debug', false);
            
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