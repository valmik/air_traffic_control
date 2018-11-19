classdef Aircraft
    %Aircraft holds the properties of a single aircraft
    %   Properties include current state as well as cost parameters and
    %   dynamics function
    
    properties
        nx; % state size
        nu; % input size
        Q; % state stage cost
        R; % input stage cost
        P; % state terminal cost
        state; % current state of the aircraft. The first two must by x, y position
        desired_state; % desired state of the aircraft. The first two must be x, y position
        state_upper_bounds; % upper bounds on state
        state_lower_bounds; % lower bounds on state
        input_upper_bounds; % upper bounds on input
        input_lower_bounds; % lower bounds on input
        state_constraints_a; % linear constraint matrix on state
        state_constraints_b; % linear constraint matrix on state
        input_constraints_a; % linear constraint matrix on state
        input_constraints_b; % linear constraint matrix on state
        nonlinear_constraints; % nonlinear constraint function, (valid if <= 0)
        linear_dynamics_a; % linear dynamics matrix (state part)
        linear_dynamics_b; % linear dynamics matrix (input part)
        nonlinear_dynamics; % nonlinear dynamics (function of x, u)
    end
    
    methods
        function obj = Aircraft()
            % Empty constructor
        end
        function cons = yalmip_constraints(obj, x_yalmip, u_yalmip)
            % Generates a constraint matrix given the variables inside
            % x_yalmip should have size (nx, N+1)
            % u_yalmip should have size (nu, N)
            
            if (size(x_yalmip, 1) ~= obj.nx)
                return
            end
            if (size(u_yalmip, 1) ~= obj.nu)
                return
            end
            N = size(u_yalmip, 2);
            
            cons = [[x_yalmip(:, 1) == obj.state]:'initial state'];
            cons = [cons, obj.final_constraint(x_yalmip, u_yalmip)];

            if (~isempty(obj.state_upper_bounds))
                for k = 1:N+1
                    cons = [cons, [obj.state_lower_bounds <= ...
                        x_yalmip(:, k) <= ...
                        obj.state_upper_bounds]:'state bounds'];
                end
            end
            if (~isempty(obj.input_upper_bounds))
                for k = 1:N
                    cons = [cons, [obj.input_lower_bounds <= ...
                        u_yalmip(:, k) <= ...
                        obj.input_upper_bounds]:'input bounds'];
                end
            end
            if (~isempty(obj.state_constraints_a))
                for k = 1:N+1
                    cons = [cons, [obj.state_constraints_a * ...
                        x_yalmip(:, k) <= ...
                        obj.state_constraints_b]:'linear state constraints'];
                end
            end
            if (~isempty(obj.input_constraints_a))
                for k = 1:N
                    cons = [cons, [obj.input_constraints_a * ...
                        u_yalmip(:, k) <= ...
                        obj.input_constraints_b]:'linear input constraints'];
                end
            end
            if (~isempty(obj.nonlinear_constraints))
                for k = 1:N
                    cons = [cons, [obj.nonlinear_constraints(x_yalmip(:,k), ...
                        u_yalmip(:,k)) <= 0 ]:'nonlinear constraints'];
                end
            end
            if (~isempty(obj.linear_dynamics_a))
                for k = 1:N
                    cons = [cons, [x_yalmip(:, k+1) == ...
                        obj.linear_dynamics_a * x_yalmip(:, k) + ...
                        obj.linear_dynamics_b * u_yalmip(:, k) ...
                        ]:'linear dynamics'];
                end
            end
            if (~isempty(obj.nonlinear_dynamics))
                for k = 1:N
                    cons = [cons, [x_yalmip(:, k+1) == ...
                        obj.nonlinear_dynamics(x_yalmip(:, k), ...
                        u_yalmip(:, k))]:'nonlinear dynamics'];
                end
            end
        end
        function final = final_constraint(obj, x_yalmip, u_yalmip)
            final = [[x_yalmip(:, end) == obj.desired_state]:'final state'];
        end
    end
    
end

