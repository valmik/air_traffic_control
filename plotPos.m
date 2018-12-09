function out = plotPos(params)
numPlanes = numel(params.aircraft_list);
figure(1); clf;
for i = 1:numPlanes
    plane = params.aircraft_list(i);
    state = value(plane.x_yalmip);
    pos = state(1:2,:);
    plot(pos(1,:),pos(2,:),'+-');
    hold on
end
limVal = 10E3;
xlim(limVal*[-1 1]);
ylim(limVal*[-1 1]);
grid