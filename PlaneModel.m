classdef PlaneModel < Aircraft
    % The model for this aircraft is basically a unicycle model
    properties
       % Aircraft Parameter
        m = 64.5E3; % Mass of aircraft (kg) From Airbus a320
        S = 122.6*2; % Surface of the Wings (m^2) From Airbus a320
        Cd = 0.026; % Coefficient of Drag From A MODEL PREDICTIVE CONTROL APPROACH TO AIRCRAFT MOTION CONTROL
        Cl = 0.24; % Coefficient of Lift From A MODEL PREDICTIVE CONTROL APPROACH TO AIRCRAFT MOTION CONTROL
        rho = 1.225; % Air Density (kg/m^3) at ground level
       thrustMax;
       thrustMin;   
       bankLim;
    end
    methods
        function obj = PlaneModel(id, x0)
           % Creates a plane model with default constraints and dynamics
           % x0 should be a 5x1 state array (x,y,v,psi,Fuel)
           % the inputs are phi, T (bank angle, thrust)
           
           if all(size(x0) ~= [5 1])
               disp('invalid state');
           end
           obj.id = id;
           obj.state = x0;
           obj.nx = 5;
           obj.nu = 2;
           distCost = 5;
           veloCost = 0; psiCost = 0; fuelCost = 0;
           obj.Q = diag([distCost,distCost,veloCost,psiCost,fuelCost]); %stage
           bankAngleCost = 1;
           obj.bankLim = pi/3;
           thrustCost = 1;
           obj.thrustMax = 2*112.5E3; 
           obj.thrustMin = obj.thrustMax/200;
           obj.R = diag([bankAngleCost, thrustCost]); %input
           obj.P = zeros(obj.nx); %final
           xylim = 300E3;
           maxV = 900/3.6; minV = 200/3.6;
           obj.state_upper_bounds = [xylim; xylim; maxV; Inf; 10^10]; % Units [m,m,m/s,rad,N]
           obj.state_lower_bounds = [-xylim; -xylim; minV; -Inf;0];
           obj.input_upper_bounds = [obj.bankLim; obj.thrustMax];              % Units [rad, N]
           obj.input_lower_bounds = [-obj.bankLim; obj.thrustMin];
           
%            obj.nonlinear_constraints = @(x, u) 0;
                                              %(x,y,v,psi,Fuel)  (bank angle, thrust)
           g = 9.8; 
           eta = 10; %arbitrary
           Kd = obj.Cd*obj.rho*obj.S/2;  
           obj.nonlinear_dynamics = @(x,u,t) x+[x(3)*cos(x(4));
                                                x(3)*sin(x(4));
                                                -Kd*(x(3).^2)/obj.m + u(2)/obj.m;
                                                 g*tan(u(1))/x(3); 
                                                -eta*u(2)]*t;
%https://www.politesi.polimi.it/bitstream/10589/114191/1/Tesi.pdf%
% pg 9
           obj.radius = 17.4; %m, half wingspan of a320
        end
        
    end
    
end
