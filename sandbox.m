%%
clear; clc
phi = 0; g = 9.81;aL = g; 
bankLim = pi/3; minV = 200/3.6; xylim = 300E3;
UA = [cos(phi) sin(phi);
    -cos(phi) -sin(phi);
    -sin(phi)/g cos(phi)/g;
     sin(phi)/g -cos(phi)/g];
UB = [aL; aL; tan(bankLim); tan(bankLim)];
U = Polyhedron('H',[UA UB]);
XA = [0 0 -cos(phi) -sin(phi);
      1 0   0         0;
      0 1   0         0];
XB = [-minV; xylim; xylim];
X = Polyhedron('H',[XA XB]);

A = [zeros(2) eye(2); zeros(2) zeros(2)];
B = [zeros(2); eye(2)];
HsA = [eye(4); -eye(4)];
HsB = [-99; 1; 11; 10;   101; 1; -9; 8];
S = Polyhedron('H',[HsA HsB]);
postS = post(A,B,S,U);

figure(123);clf
b = S.projection(1:2);
b.plot();
figure(456);clf;
a = postS.projection(1:2);
a.plot();


%%
clear;figure(1);clf
A = eye(2); 
B = .5*ones(2,1);
H = [eye(2); -eye(2)];
h = ones(4,1);
S = Polyhedron('H',[H 5*h]);
U = Polyhedron('H',[H 5*h]);
S.plot()
hold on
preset = Pre(A,B,S,U);
figure(2);clf
preset.plot()
%%
clear
Ac = [0 1;2.5 -0.05];
Bc = [0;2.5];
nx = size(Ac,2);
nu = size(Bc,2);
% Check real part of eigs in open left-half plane
% Linear, Time-Invariant, Discrete-Time Dynamical System
Ts = 0.1;
[A, B] = c2d(Ac,Bc,Ts);
umin = -pi/10;           % [rad/s^2] acceleration
umax = +pi/10;           % [rad/s^2] acceleration
% umin = 0;           % [rad/s^2] acceleration
% umax = 0;           % [rad/s^2] acceleration
model.x.min = [-pi/4;-pi/2];    % [rad, rad/s]  position, velocity
model.x.max = [+pi/4;+pi/2];    % [rad, rad/s]  position, velocity

% constraint sets represented as polyhedra
X = Polyhedron('lb',model.x.min,'ub',model.x.max);
U = Polyhedron('lb',umin,'ub',umax);
figure(1);clf;
X.plot();
X.computeVRep();
preset = Pre(A,B,X,U);
prepreset = Pre(A,B,preset,U);
prepreset.computeVRep();
figure(2); clf
preset.plot();
preset.computeVRep();
verts = preset.V';
% vert2 = zeros(numel(verts)
A*verts;
A*prepreset.V'
preset.V'
test = Polyhedron('V',preset.V);
figure(123);clf;
postprepreset = post(A,B,prepreset,U);
postprepreset.plot()
figure(456);clf
preset.plot()



function PreS = Pre(A,B,S,U)
% INPUT ARGUMENTS
%   A : LTI, Discrete-time System x(k+1) = A*x(k) + B*u(k)
%   B : LTI, Discrete-time System x(k+1) = A*x(k) + B*u(k)
%   S : Target Set
%   U : Input Constraint Set
% OUTPUT ARGUMENTS
%  PreS : Pre set for a driven system

nx = size(A,2);
nu = size(B,2);

H  = S.H(:,1:nx);
h  = S.H(:,nx+1);
Hu = U.H(:,1:nu);
hu = U.H(:,nu+1);
He = S.He(:,1:nx);
he = S.He(:,nx+1);

XU_H = [H*A H*B; zeros(size(Hu,1),nx) Hu];
XU_h = [h;hu];

XU = Polyhedron('H',[XU_H, XU_h],'He',[He*A, He*B, he]);
PreS = XU.projection(1:nx);
end