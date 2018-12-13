function [] = plotPos(params)
figure(1); clf

keySet = keys(params.aircraft_list);

for key = keySet
    plane = params.aircraft_list(key{1});
    state = value(plane.x_yalmip);
    pos = state(1:2,:);
    plot(pos(1,:),pos(2,:),'+-', ...
        'color', params.color_list(key{1}), ...
        'DisplayName', key{1});
    hold on
end
limVal = 10E3;
xlim(limVal*[-1 1]);
ylim(limVal*[-1 1]);
grid
% legend(keySet)
legend('-DynamicLegend')

figure(10);
hold on
for key = keySet
    plane = params.aircraft_list(key{1});
    state = plane.state;
    pos = state(1:2,:);
    plot(pos(1,:),pos(2,:),'+-', ...
        'color', params.color_list(key{1}), ...
        'DisplayName', key{1});
end
limVal = 10E3;
xlim(limVal*[-1 1]);
ylim(limVal*[-1 1]);
grid
hLegend = legend('-DynamicLegend');
hLegend.String = unique(hLegend.String);

pause(0.1)