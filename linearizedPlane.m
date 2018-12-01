classdef linearizedPlane < Aircraft
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
        function obj = PlaneModel(id, x0, Ts)
           % Creates a plane model with default constraints and dynamics
           % x0 should be a 5x1 state array (x,y,v,psi,Fuel)
           % the inputs are phi, T (bank angle, thrust)
           obj.nx = 4; obj.nu = 2;
           if all(size(x0) ~= [4 1])
               error('invalid state');
           end
           obj.state = x0;
           obj.id = id;
           obj.sI = 1;
           
           distCost = 5;
           veloCost = 0; psiCost = 0; fuelCost = 0;
%            obj.Q = diag([distCost,distCost,veloCost,psiCost,fuelCost]); %stage
           bankAngleCost = 1;
           obj.bankLim = pi/3;
           thrustCost = 1;
           obj.thrustMax = 2*112.5E3; 
           obj.thrustMin = obj.thrustMax/200;
%            obj.R = diag([bankAngleCost, thrustCost]); %input
%            obj.P = zeros(obj.nx); %final
           xylim = 300E3;
           maxV = 900/3.6; minV = 200/3.6;
           g = 9.8; 
           eta = 10; %arbitrary
           
           obj.linear_dynamics_a = [eye(2) Ts*eye(2); 
                                    zero(2) eye(2)];
           obj.linear_dynamics_b = [(Ts.^2)*eye(2)/2; 
                                     Ts*eye(2)];
           
%https://www.politesi.polimi.it/bitstream/10589/114191/1/Tesi.pdf%
% pg 28ish
           obj.radius = 17.4; %m, half wingspan of a320
        end
        function out = getState(i)
           %convert from linearized states to physical states
           stateArr = value(obj.x_yalmip);
           out = zeros(obj.nx,1);
           out(1:2) = stateArr(1:2,i);
           out(3) = sqrt(obj.stateArr(4,i).^2 + obj.stateArr(5,i).^2);
           out(4) = atan2(obj.stateArr(5,i),obj.stateArr(4,i));
        end   
    end
end
