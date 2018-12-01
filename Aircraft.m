classdef Aircraft < handle
    %Aircraft holds the properties of a single aircraft
    %   Properties include current state as well as cost parameters and
    %   dynamics function
    
    properties
        id; % airplane id number
        nx; % state size
        nu; % input size
        
        state; % current state of the aircraft.
        
        % Cost
        Q; % state stage cost
        R; % input stage cost
        P; % state terminal cost
        
        % Bounds
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
  
        % Other parameters
        radius; % min safe radius
        
        % Should be set up during yalmip setup
        N; % horizon
        x_yalmip; % sdpvar for x
        u_yalmip; % sdpvar for u
        yalmip_constraints; % constraints
        yalmip_cost; % cost
    end
    
    methods
        function obj = Aircraft()
            % Empty constructor
        end
        
        function obj = setup_yalmip(obj, N, timesteps)
            % This function creates yalmip variables for optimization and
            % creates the individual constraints and costs for the model
            % these should then be combined with those of other planes for
            % overall optimization
            obj.N = N;
            obj.x_yalmip = sdpvar(obj.nx, obj.N + 1);
            obj.u_yalmip = sdpvar(obj.nu, obj.N);
            obj = obj.set_yalmip_constraints(timesteps);
            obj = obj.set_yalmip_cost();
        end
        
        function obj = set_yalmip_constraints(obj, timesteps)
            % Generates a constraint matrix given the variables inside
            % x_yalmip should have size (nx, N+1)
            % u_yalmip should have size (nu, N)
            % timesteps is either a sdpvar or double array of length N. It's the
            % length of each timestep in the discretized dynamics function
            
            cons = [(obj.x_yalmip(:, 1) == obj.state):[obj.id, ' initial state']];
            % State bounds
            if (~isempty(obj.state_upper_bounds))
                for k = 1:obj.N+1
                    cons = [cons, [obj.state_lower_bounds <= obj.x_yalmip(:, k) <= ...
                        obj.state_upper_bounds]:[obj.id, ' state bounds']];
                end
            end
            % Input bounds
            if (~isempty(obj.input_upper_bounds))
                for k = 1:obj.N
                    cons = [cons, [obj.input_lower_bounds <= obj.u_yalmip(:, k) <= ...
                        obj.input_upper_bounds]:[obj.id, ' input bounds']];
                end
            end
            
            % Linear state constraints
            if (~isempty(obj.state_constraints_a))
                for k = 1:obj.N+1
                    cons = [cons, [obj.state_constraints_a * obj.x_yalmip(:, k) <= ...
                        obj.state_constraints_b]:[obj.id, ' linear state constraints']];
                end
            end
            
            % Linear input constraints
            if (~isempty(obj.input_constraints_a))
                for k = 1:obj.N
                    cons = [cons, [obj.input_constraints_a * obj.u_yalmip(:, k) <= ...
                        obj.input_constraints_b]:[obj.id, ' linear input constraints']];
                end
            end
            
            % Nonlinear constraints
            if (~isempty(obj.nonlinear_constraints))
                for k = 1:obj.N
                    cons = [cons, ...
                      [obj.nonlinear_constraints(obj.x_yalmip(:,k),obj.u_yalmip(:,k)) <= 0 ]:[obj.id, ' nonlinear constraints']];
                end
            end
            
%             %Linear Dynamics
            if (~isempty(obj.linear_dynamics_a))
                for k = 1:obj.N
                    cons = [cons, [obj.x_yalmip(:, k+1) == obj.x_yalmip(:, k) + ...
                        obj.linear_dynamics_a * obj.x_yalmip(:, k) * timesteps(k) + ...
                        obj.linear_dynamics_b * obj.u_yalmip(:, k) * timesteps(k)...
                        ]:[obj.id, ' linear dynamics']];
                end
            end
            
            % Nonlinear Dynamics
            if (~isempty(obj.nonlinear_dynamics))
                for k = 1:obj.N
                    cons = [cons, [obj.x_yalmip(:, k+1) == ...
                        obj.nonlinear_dynamics(obj.x_yalmip(:, k), obj.u_yalmip(:, k), timesteps(k)) ...
                        ]:[obj.id, ' nonlinear dynamics']];
                end
            end
            
            obj.yalmip_constraints = cons;
        end
        
        function obj = set_yalmip_cost(obj)
            obj.yalmip_cost = obj.x_yalmip(:,obj.N+1)'*obj.P*obj.x_yalmip(:,obj.N+1);
            for k = 1:obj.N
                obj.yalmip_cost = obj.yalmip_cost + ...
                    obj.x_yalmip(:,k)'*obj.Q*obj.x_yalmip(:,k) + ...
                    obj.u_yalmip(:,k)'*obj.R*obj.u_yalmip(:,k);
            end
        end
    end
    
end

