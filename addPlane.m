function params = addPlane(plane,params,N )

numOtherPlanes = numel(params.aircraft_list);
params.aircraft_list = [params.aircraft_list plane.setup_yalmip(N,NaN)];
%adds to aircraft array

params.obj = params.obj + plane.yalmip_cost;
%adds plane distance cost over N timesteps (PQR costs)

x1 = plane.x_yalmip;
pos1 = x1(1:2);
% for i = 1:N
%     for j = 1:numOtherPlanes
%         x2 = params.aircraft_list(j).x_yalmip;
%         pos2 = x2(1:2); %get xy position for other plane
%         distCost = sum((pos2-pos1).^2); %distance squared
%         params.obj = params.obj - 100*distCost;
%     end
% end