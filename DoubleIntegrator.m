classdef DoubleIntegrator < Aircraft
    % The model for this aircraft is a 2d double integrator
    
    methods
        function obj = DoubleIntegrator(x0, xf)
           % Creates a double integrator with default constraints and dynamics
           % x0 should be a 5x1 state array (x,y,dx,dy,f)
           
           if (any(size(x0)-[5,1]))
               x0 = zeros(5,1);
           end
           obj.state = x0;
           obj.desired_state = xf;
           obj.nx = 5;
           obj.nu = 2;
           obj.Q = diag([1,1,1,1,-1]);
           obj.R = eye(2);
           obj.P = diag([1,1,1,1,-1]);
           obj.state_upper_bounds = [10;10;10;10;100];
           obj.state_lower_bounds = [-10;-10;-10;-10;0];
           obj.input_upper_bounds = [1; 1];
           obj.input_lower_bounds = [-1; -1];
           obj.linear_dynamics_a = [0, 0, 1, 0, 0; ... % x
                                    0, 0, 0, 1, 0; ... % y
                                    0, 0, 0, 0, 0; ... % dx
                                    0, 0, 0, 0, 0; ... % dy
                                    0, 0, 0, 0, 0]; % fuel
           obj.linear_dynamics_b = [0, 0; ...
                                    0, 0; ...
                                    1, 0; ...
                                    0, 1; ...
                                    -1, -1];
        end
        
        function final = final_constraint(obj, x_yalmip, u_yalmip)
            final = [[x_yalmip(1:4, end) == obj.desired_state(1:4)]:'final state'];
        end
    end
    
end