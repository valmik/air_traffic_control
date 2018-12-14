classdef linearizedPlane < Aircraft
    properties
       % Aircraft Parameter
        m = 64.5E3; % Mass of aircraft (kg) From Airbus a320
        S = 122.6*2; % Surface of the Wings (m^2) From Airbus a320
        Cd = 0.053; % Coefficient of Drag From Aircraft Performance Parameter Estimation using Global ADS-B and Open Data
        Cl = 0.466; % Coefficient of Lift From Aircraft Performance Parameter Estimation using Global ADS-B and Open Data
        rho = 1.225; % Air Density (kg/m^3) at ground level
       thrustMax;
       thrustMin;   
       bankLim;
       psi
       V
    end
    methods
        function obj = linearizedPlane(id, x0, psi, V, Ng)
           % Creates a plane model with default constraints and dynamics
           obj.nx = 4; obj.nu = 2;
           if all(size(x0) ~= [4 1])
               error('invalid state');
           end
           obj.psi = psi;
           obj.V = V;
           obj.simCounter = 1;
           obj.stateArr = zeros(obj.nx,Ng);
           obj.stateArr(:,obj.simCounter) = x0;
           obj.state = obj.stateArr(:,obj.simCounter);
           obj.radius = 17.4; %m, half wingspan of a320
           
           obj.id = id;
           distC = 50; %cost for dist from origin
           obj.Q = diag([distC,distC,0,0]); %stage
           obj.R = .2*eye(2); %input
           obj.P = diag([5000,5000,0,0]); %final
           
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
           obj.linear_dynamics_a = [zeros(2) eye(2);       %checked
                                    zeros(2) zeros(2)];    %checked
           obj.linear_dynamics_b = [zeros(2); eye(2)];     %checked
           obj.nonlinear_constraints = @(x, u) [(x(3).^2 + x(4).^2 - maxV.^2);
               ((obj.m*(u(1)*cos(obj.psi)+u(2)*sin(obj.psi))+Kd*obj.V) - obj.thrustMax);
                                                (obj.thrustMin - (obj.m*(u(1)*cos(obj.psi)+u(2)*sin(obj.psi))+Kd*obj.V))];
%                                   ; removed
%                                                 nonlinear constraint
           obj.state_constraints_a = [0  0 -cos(obj.psi) -sin(obj.psi); %checked
                                      1  0   0         0;
                                      0  1   0         0];
           obj.state_constraints_b = [-minV; xylim; xylim];
           obj.input_constraints_a = [cos(obj.psi) sin(obj.psi);
                                      -cos(obj.psi) -sin(obj.psi);
                                      -sin(obj.psi)/g cos(obj.psi)/g;
                                      sin(obj.psi)/g -cos(obj.psi)/g];
           obj.input_constraints_b = [aL; aL; tan(obj.bankLim); tan(obj.bankLim)];
%            obj.input_constraints_a = [cos(obj.psi) sin(obj.psi);
%                                       -cos(obj.psi) -sin(obj.psi);];
%            obj.input_constraints_b = [aL; aL];
           %https://www.politesi.polimi.it/bitstream/10589/114191/1/Tesi.pdf%
% pg 28ish
        end
        function [X,U] = constrPoly(obj)
            X = Polyhedron('H',[obj.state_constraints_a obj.state_constraints_b]);
            U = Polyhedron('H',[obj.input_constraints_a obj.input_constraints_b]);
            fprintf("psi: %0.2f V: %0.2f\n",[obj.psi obj.V]);
        end
        function [state, input] = getState(obj,i)
           %convert from linearized states to physical states. see link at
           %dynamics constraint for reference.
           stateArr = value(obj.x_yalmip);
           state = zeros(obj.nx,1);
           state(1:2) = stateArr(1:2,i); %xy
           state(3) = sqrt(obj.stateArr(3,i).^2 + obj.stateArr(4,i).^2); %v
           state(4) = atan2d(obj.stateArr(4,i),obj.stateArr(3,i)); %psi
           input = zeros(obj.nu,1);
           input(1) = obj.inputArr(1)*cos(obj.psi) + obj.inputArr(2)*sin(obj.psi);
           input(2) = atand((1/9.81)*(-obj.inputArr(1)*sin(obj.psi)+obj.inputArr(2)*cos(obj.psi)));
        end
        function out = recordAndAdvanceState(obj)
            input = value(obj.u_yalmip(:,1)); %gets first optimal input
            obj.inputArr(:,obj.simCounter) = input; %stores optimal input at global timestep index
            nextState = value(obj.x_yalmip(:,2)); %gets optimal next state...this is wrong actually
            obj.simCounter = obj.simCounter + 1; %advance sim counter
            obj.stateArr(:,obj.simCounter) = nextState; %this should be set to currstate + optimal input
            obj.state = nextState; %sets current state of plane to updated state
%             obj.state = [nextState(1); nextState(2); 200; -100];
            prevPsi = obj.psi;
            obj.psi = atan2(obj.state(4),obj.state(3)); %sets current heading angle
            obj.V = sqrt(obj.state(3).^2 + obj.state(4).^2);
            
            obj.setConstraints(); %updates constraints based on new psi
        end
    end
end
