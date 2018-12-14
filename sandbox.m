%% test linearizedPlane

%% testing postNL
clear; clc
Ts = .1; %timestep
%initial set parameters 
dx = [.99 1.01];
x = -1*dx;
y = 1*dx;
V = 200.*dx;
psi = 0*dx;

maxV = 900/3; minV = 200/3.6;
%put initial set into a polyhedron
HsA = [eye(4); -eye(4)];
HsB = [max(x);  max(y); max(V); max(psi);
      -min(x); -min(y); -min(V); -min(psi)];
S = Polyhedron('H',[HsA HsB]); %state set
S.computeVRep(); %want vertices representation

postS = postNL(S,Ts,maxV,minV);
figure(123);clf
for i = 1:20
    tic
    [postS, flag] = postNL(postS,Ts,maxV,minV);

%     check = a.distance([0;0]); %uses built-in distance function: slow
%     if check.dist < 1
%         fprintf("arrived\n");
%     end
    time = toc;
    fprintf("TS: %d verts: %d time: %0.2f arrive: %d \n",[i size(postS.V,1) time flag]);
    figure(123);
    subplot(2,1,1);
    a = postS.projection(1:2);
    a.plot('color','b','alpha',.3);
    hold on
    xlabel('xpos');
    ylabel('ypos');
    xlim(.5E3*[-1 1]);
    ylim(.5E3*[-1 1]);
    subplot(2,1,2);
    b = postS.projection(3:4);
    b.plot('color','b','alpha',.3); hold on
    xlabel('V');
    ylabel('phi');
    pause
end

%%
clear; clc
phi = 0; 
g = 9.81; aL = g; 
bankLim = pi/3; 
maxV = 900/3; minV = 200/3.6; xylim = 300E3;
UA = [cos(phi) sin(phi);
    -cos(phi) -sin(phi);
    -sin(phi)/g cos(phi)/g;
     sin(phi)/g -cos(phi)/g];
UB = [aL*2; .5*aL; tan(bankLim); tan(bankLim)];
U = Polyhedron('H',[UA UB]);
XA = [0 0 -cos(phi) -sin(phi);
      0 0  cos(phi) sin(phi);
      1 0   0         0;
      0 1   0         0];
XB = [-minV; maxV; xylim; xylim];
X = Polyhedron('H',[XA XB]);
Ts = 2;
A = [zeros(2) eye(2); zeros(2) zeros(2)];
B = ([zeros(2); eye(2)]);

x = -5000*[.99 1.01];
% x = -5000*[.101 .99];
y = 1*[.99 1.01];

Vx = 200.*[.99 1.01];
% Vx = 200.*[.99 .101];
Vy = 1*[.99 1.01];

HsA = [eye(4); -eye(4)];
HsB = [max(x);  max(y); max(Vx); max(Vy);
      -min(x); -min(y); -min(Vx); -min(Vy)];
S = Polyhedron('H',[HsA HsB]);
postS = post(A,B,S,U,X,Ts,maxV,minV);
a = postS.projection(1:2);
figure(123);clf
a.plot(); hold on
for i = 1:20
    postS = post(A,B,postS,U,X,Ts,maxV,minV);
    figure(123);
    subplot(2,1,1);
    a = postS.projection(1:2);
    a.plot('color','b','alpha',.3);
    hold on
    xlabel('xpos');
    ylabel('ypos');
    xlim(10E3*[-1 1]);
    ylim(10E3*[-1 1]);
    subplot(2,1,2);
    b = postS.projection(3:4);
    b.plot('color','b','alpha',.3); hold on
    xlabel('xvel');
    ylabel('yvel');
%     clc
%     pause
    check = a.distance([0;0]);
    if check.dist < 1
        disp('arrived')
    end
end


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