function [] = plotPos2(params,i)
figure(1); clf
plane_pic = imread('plane.jpg'); %Import Plane Image
% plane_pic = plane_pic(:,:)
pic_l = 1500/2;                  %Plane Image size used in plotting
keySet = keys(params.aircraft_list);
% persistent i                     %Set up Time step inside the function
% if isempty(i)                    %Initialize the persistent variable
%     i = 0;
% end
% i = i+1;
for key = keySet
    plane = params.aircraft_list(key{1});
    state = value(plane.x_yalmip);
    pos = state(1:2,:);
    plot(pos(1,:),pos(2,:),'+-', ...
        'color', params.color_list(key{1}), ...
        'DisplayName', key{1});
    hold on
    statei = plane.getState(i); %Get Current State
    theta = rad2deg(-pi/2)-statei(4);   %Degrees to rotate
    pic = spinPlanePic(theta);
    alpha = (pic(:,:,1)<=150);          %Defining transparency
    pic = image([pos(1,1)-pic_l pos(1,1)+pic_l], [pos(2,1)-pic_l pos(2,1)+pic_l],pic);  %Plot the plane at (xmin, xmax), (ymin, ymax)
    set(pic,'AlphaData', alpha)                 %Set up transparency
    viscircles([0 0],params.lzdia);
end

limVal = 10E3;
xlim(limVal*[-1 1]);
ylim(limVal*[-1 1]);
grid
% legend(keySet)
legend('-DynamicLegend')

figure(10);clf
hold on
for key = keySet
    plane = params.aircraft_list(key{1});
    state = plane.state;
    pos = state(1:2,:);
    plot(pos(1,:),pos(2,:),'+-', ...
        'color', params.color_list(key{1}), 'DisplayName', key{1});
    statei = plane.getState(i); %Get Current State
    theta = rad2deg(-pi/2)-statei(4);   %Degrees to rotate
    pic = spinPlanePic(theta);
    alpha = (pic(:,:,1)<=150);          %Defining transparency
    pic = image([pos(1,1)-pic_l pos(1,1)+pic_l], [pos(2,1)-pic_l pos(2,1)+pic_l],pic);  %Plot the plane at (xmin, xmax), (ymin, ymax)
    set(pic,'AlphaData', alpha)                 %Set up transparency
    viscircles([0 0],params.lzdia);
end
limVal = 10E3;
xlim(limVal*[-1 1]);
ylim(limVal*[-1 1]);
grid
hold on
hLegend = legend('-DynamicLegend');
hLegend.String = unique(hLegend.String);

pause(0.1)