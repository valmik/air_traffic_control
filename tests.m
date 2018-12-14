%%
figure(123); clf
plane_pic = imread('plane.jpg'); %Import Plane Image
plane_pic = padarray(plane_pic,[50 50],1000,'both');
imshow(plane_pic)

theta = 60;
pic = imrotate(plane_pic, theta,'crop')
tl = round(325/4,0);
% pic = imcrop(pic,[0 0 5000 5000])
% pic = imclearborder(pic);
% imshow(pic)
border = 50;
pic(1:border,:,:) = 1000;
pic(:,end-border:end,:) = 1000;
pic(:,1:border,:) = 1000;
pic(end-border:end,:) = 1000;
imshow(pic)

%% test linearizedPlane
clear
v = 300; psi1 = 0; Ng = 100;xylim = 4*10^4;
x0a = [-8000; 0;v*cos(psi1);v*sin(psi1)];
a = linearizedPlane('1',x0a,psi1,v,xylim,Ng);
[X,U] = a.constrPoly();
figure(1); clf
subplot(2,1,1);
xy = X.projection(1:2);
xy.plot(); title('xy');
subplot(2,1,2);
Vxy = X.projection(3:4);
Vxy.plot(); title('Vxy');
figure(2); clf;
U.plot(); title('input');


%%
params = struct();
params.constr = [];
params.obj = 0;
params.aircraft_list = [];
N = 5;
x1 = [0; 0; 0; 0];
p1 = linearizedPlane('1',x1,0,0,1);
params = addPlane(p1,params,N);
x2 = [1; 0; 0; 0];
p2 = linearizedPlane('1',x2,0,0,1);
params = addPlane(p2,params,N);
%% distcost check
clear
pos1 = [0; 0];
pos2 = [2; 2];
distCost = sqrt(sum((pos2-pos1).^2))

%% LinearizedPlane tests
clear; clc
N = 100; %steps
Ts = .1; xy = [0; 0]; V = 200; psi = pi/2; %heading angle
g = 9.8; %m/s
phi = 0; %roll angle
x0 = [xy(1); xy(2); V*cos(psi); V*sin(psi)];
a = linearizedPlane('1',x0,phi,V,Ts);
tau = 50; %vdot
gamma = tan(phi);
u1 = tau*cos(psi) - V*sin(psi)*gamma*g/V;
u2 = tau*sin(psi) + V*cos(psi)*gamma*g/V;
uArr = [u1; u2].*ones(a.nu,N-1);
xArr = zeros(a.nx,N);
xArr(:,1) = a.state; %set initial cond
for i = 2:N
    xArr(:,i) = xArr(:,i-1) + (a.linear_dynamics_a*xArr(:,i-1)+a.linear_dynamics_b*uArr(:,i-1))*Ts;
end
figure(123); clf
plot(xArr(1,:),xArr(2,:),'+');
xlim(3E3*[-1 1]);
ylim(3E3*[-.1 1]);
grid
disp('done');
%% PlaneModel tests
clear; clc
x0 = [0;2*10^3;200;-pi/2;10^9];
a = PlaneModel('1',x0);
dt = .5; %s
N = 100; %steps
uArr = [pi/6; a.thrustMax].*ones(a.nu,N-1);
xArr = zeros(a.nx,N);
xArr(:,1) = a.state; %set initial cond
for i = 2:N
    xArr(:,i) = a.nonlinear_dynamics(xArr(:,i-1),uArr(:,i-1),dt);
    if any(xArr(:,i) > a.state_upper_bounds) || any(xArr(:,i) < a.state_lower_bounds)
        disp('hold');
    end
end
figure(123); clf
plot(xArr(1,:),xArr(2,:),'+');
xlim(3E3*[-1 1]);
ylim(3E3*[-1 1]);
grid
