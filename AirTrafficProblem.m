classdef AirTrafficProblem
    %AirTrafficProblem Contains all the info needed to solve an air traffic
    %control problem, including the overall cost and constraints
    
    properties
        aircraft_list; % list of aircraft
        N;
%         nx; % array of nx for each aircraft
%         nu; % array of nu for each aircraft
%         x_yalmip;
%         u_yalmip;
        time_yalmip; % timesteps 
        cost;
        constraints;
    end
    
    methods
        function obj = AirTrafficProblem(list, N)
            obj.aircraft_list = list;
            obj.N = N;
            obj = obj.GetYalmip();
            obj = obj.GetCost();
            obj = obj.GetConstraints();
        end
        
        function obj = GetYalmip(obj)
%             obj.nx = zeros(numel(obj.aircraft_list));
%             obj.nu = zeros(numel(obj.aircraft_list));
%             for i = 1:numel(obj.aircraft_list)
%                 obj.nx(i) = obj.aircraft_list(i).nx;
%                 obj.nu(i) = obj.aircraft_list(i).nu;
%             end
%             obj.x_yalmip = sdpvar(sum(sum(obj.nx)), obj.N+1);
%             obj.u_yalmip = sdpvar(sum(sum(obj.nu)), obj.N);
    
            for i = 1:numel(obj.aircraft_list)
               obj.aircraft_list(i) = obj.aircraft_list(i).setup_yalmip(obj.N); 
            end
        end
        
        function obj = GetCost(obj)
%             P_list = cell(size(obj.aircraft_list));
%             Q_list = cell(size(obj.aircraft_list));
%             R_list = cell(size(obj.aircraft_list));
%             for i = 1:numel(obj.aircraft_list)
%                 P_list{i} = obj.aircraft_list(i).P;
%                 Q_list{i} = obj.aircraft_list(i).Q;
%                 R_list{i} = obj.aircraft_list(i).R;
%             end
%             P = blkdiag(P_list{:});
%             Q = blkdiag(Q_list{:});
%             R = blkdiag(R_list{:});
            obj.cost = 0;
            for i = 1:numel(obj.aircraft_list)
                obj.cost = obj.cost + obj.aircraft_list(i).yalmip_cost;
            end
            obj = obj.CollisionCost();
        end
        function obj = GetConstraints(obj)
            obj.constraints = [];
            for i = 1:numel(obj.aircraft_list)
               obj.constraints = [obj.constraints, ...
                   obj.aircraft_list(i).yalmip_constraints];
            end
%             obj.constraints = obj.aircraft_list(1).yalmip_constraints( ...
%                 obj.x_yalmip(1:obj.nx(1), :), ...
%                 obj.u_yalmip(1:obj.nu(1), :));
%             if numel(obj.aircraft_list) <= 1
%                 return
%             end
%             for i = 2:numel(obj.aircraft_list)
%                 obj.constraints = [obj.constraints, ...
%                     obj.aircraft_list(i).yalmip_constraints(...
%                         obj.x_yalmip( ...
%                             (sum(sum(obj.nx(1:(i-1)))) + 1):sum(sum(obj.nx(1:i))), :), ...
%                         obj.u_yalmip( ...
%                             (sum(sum(obj.nu(1:(i-1)))) + 1):sum(sum(obj.nu(1:i))), :))];
%             end
            obj = obj.CollisionConstraints();
        end
        
        
        function obj = CollisionCost(obj)
            for i = 1:numel(obj.aircraft_list)
                for j = (i+1):numel(obj.aircraft_list)
                    vector_diff = obj.aircraft_list(i).x_yalmip - obj.aircraft_list(j).x_yalmip;
                    radius = max(obj.aircraft_list(i).radius^2, obj.aircraft_list(j).radius^2);
                    obj.cost = obj.cost - log(radius - vector_diff(1)^2 + vector_diff(2)^2);
                end
            end
        end
        
        function obj = CollisionConstraints(obj)
            if numel(obj.aircraft_list) < 2
                return
            end
            
            for i = 1:numel(obj.aircraft_list)
                for j = (i+1):numel(obj.aircraft_list)
                    vector_diff = obj.aircraft_list(i).x_yalmip - obj.aircraft_list(j).x_yalmip;
                    radius = max(obj.aircraft_list(i).radius.^2, obj.aircraft_list(j).radius.^2);
                    obj.constraints = [obj.constraints, ... 
                        ((vector_diff(1)^2 + vector_diff(2)^2) >= radius ...
                        ):[obj.aircraft_list(i).id, ' ', obj.aircraft_list(j).id, ' collision constraint']];
                end
            end
            return
        end
        
        function [] = PlotSolution(obj)
            
            figure()
            hold on
            legend_str = cell(size(obj.aircraft_list));
            for i = 1:numel(obj.aircraft_list)
                x_opt = double(obj.aircraft_list(i).x_yalmip);
                plot(x_opt(1, :), x_opt(2, :), '--*')
                legend_str{i} = obj.aircraft_list(i).id;
            end
            title('Position plot')
            legend(legend_str)
        end
        
        
        
        
        function [x_opt, u_opt, exitval_opt] = RunMPC(obj)
            options = sdpsettings('solver', 'fmincon','verbose',3, 'showprogress', 5);
            
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
            
            tic
                exitval_opt = optimize(obj.constraints, obj.cost, options);
            toc
            
            x_opt = [];
            u_opt = [];
            for i = 1:numel(obj.aircraft_list)
                x_opt = [x_opt, double(obj.aircraft_list(i).x_yalmip)];
                u_opt = [u_opt, double(obj.aircraft_list(i).u_yalmip)];
            end
        end
    end
    
end

