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
