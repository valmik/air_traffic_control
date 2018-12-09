classdef linearizedPlane < Aircraft
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
       phi
       V
    end
    methods
        function obj = linearizedPlane(id, x0, phi, V, Ng)
           % Creates a plane model with default constraints and dynamics
           obj.nx = 4; obj.nu = 2;
           if all(size(x0) ~= [4 1])
               error('invalid state');
           end
           obj.phi = phi;
           obj.V = V;
           obj.simCounter = 1;
           obj.stateArr = zeros(obj.nx,Ng);
           obj.stateArr(:,obj.simCounter) = x0;
           obj.state = obj.stateArr(:,obj.simCounter);
           obj.radius = 17.4; %m, half wingspan of a320
           
           obj.id = id;
           
           distC = 50; %cost for dist from origin
           

           
           obj.Q = diag([distC,distC,0,0]); %stage
           obj.R = eye(2); %input
           obj.P = zeros(obj.nx); %final
           
           obj.setConstraints();
        end
        function out = setConstraints(obj)
            xylim = 300E3;
           maxV = 900/3.6; minV = 200/3.6;
           g = 9.8; aL = g;
           Kd = (obj.Cd*obj.rho*obj.S)/2;
           obj.bankLim = pi/3;
           obj.thrustMax = 2*112.5E3; 
           obj.thrustMin = obj.thrustMax/200;
           obj.linear_dynamics_a = [zeros(2) eye(2); 
                                    zeros(2) zeros(2)];
           obj.linear_dynamics_b = [zeros(2); 
                                     eye(2)];
           obj.nonlinear_constraints = @(x, u) [x(3).^2 + x(4).^2 - maxV.^2;
                                                (obj.m*(u(1)*cos(obj.phi)+u(2)*sin(obj.phi))+Kd*obj.V) - obj.thrustMax;
                                                obj.thrustMin - (obj.m*(u(1)*cos(obj.phi)+u(2)*sin(obj.phi))+Kd*obj.V)];
           obj.state_constraints_a = [0 0 -cos(obj.phi) -sin(obj.phi);
                                      1 0   0         0;
                                      0 1   0         0];
           obj.state_constraints_b = [-minV;
                                       xylim;
                                       xylim];
           obj.input_constraints_a = [cos(obj.phi) sin(obj.phi);
                                      -cos(obj.phi) -sin(obj.phi);
                                      -sin(obj.phi)/g cos(obj.phi)/g;
                                      sin(obj.phi)/g -cos(obj.phi)/g];
           obj.input_constraints_b = [aL; aL; tan(obj.bankLim); tan(obj.bankLim)];
           %https://www.politesi.polimi.it/bitstream/10589/114191/1/Tesi.pdf%
% pg 28ish
        end
        function out = getState(obj,i)
           %convert from linearized states to physical states
           stateArr = value(obj.x_yalmip);
           out = zeros(obj.nx,1);
           out(1:2) = stateArr(1:2,i);
%            out(3) = sqrt(obj.stateArr(4,i).^2 + obj.stateArr(5,i).^2);
           out(4) = atan2(obj.stateArr(4,i),obj.stateArr(3,i));
        end
        function out = recordAndAdvanceState(obj)
            input = value(obj.u_yalmip(:,1));
            obj.inputArr(:,obj.simCounter) = input;
            nextState = value(obj.x_yalmip(:,2));
            obj.simCounter = obj.simCounter + 1;
            obj.state = nextState;
            obj.phi = atan2(obj.state(4),obj.state(3));
            obj.setConstraints(); %updates constraints based on new phi
        end
    end
end
