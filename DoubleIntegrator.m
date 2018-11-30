classdef DoubleIntegrator < Aircraft
    % The model for this aircraft is a 2d double integrator
    
    methods
        function obj = DoubleIntegrator(id, x0)
           % Creates a double integrator with default constraints and dynamics
           % x0 should be a 5x1 state array (x,y,dx,dy,f)
           if (any(size(x0)-[5,1]))
               x0 = zeros(5,1);
           end
           obj.id = id;
           obj.state = x0;
           obj.nx = 5;
           obj.nu = 2;
           obj.Q = diag([2,1,0,0,-0.01]);%
           obj.R = eye(2); %input
           obj.P = diag([5,5,1,1,0]); %final
           obj.state_upper_bounds = [10;10;10;10;100];
           obj.state_lower_bounds = [-10;-10;-10;-10;0];
           obj.input_upper_bounds = [10; 10];
           obj.input_lower_bounds = [-10; -10];
           obj.radius = .1;
           obj.nonlinear_dynamics = @(x, u, t) ...
                x + [x(3);
                 x(4);
                 u(1);
                 u(2);
                 - u(1).^2 - u(2).^2]*t;
             %            obj.linear_dynamics_a = [0, 0, 1, 0, 0; ... % x
%                                     0, 0, 0, 1, 0; ... % y
%                                     0, 0, 0, 0, 0; ... % dx
%                                     0, 0, 0, 0, 0; ... % dy
%                                     0, 0, 0, 0, 0]; % fuel
%            obj.linear_dynamics_b = [0, 0; ...
%                                     0, 0; ...
%                                     1, 0; ...
%                                     0, 1; ...
%                                     -1, -1]; % Doesn't work because fuel
%                                     increases when it accelerates
%                                     backwards
        end
        
    end
    
end