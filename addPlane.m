function params = addPlane(plane,params,N )

otherPlanes = keys(params.aircraft_list);

params.aircraft_list(plane.id) = plane.setup_yalmip(N,NaN);
%adds to aircraft array

params.color_list(plane.id) = rand(1,3);

params.costs(plane.id, plane.id) = plane.yalmip_cost;

%adds plane distance cost over N timesteps (PQR costs)

x1 = plane.x_yalmip;
for key = otherPlanes
    x2 = params.aircraft_list(key{1}).x_yalmip;
    distCost = 0;
    for i = 1:N
        distCost = distCost + sum((x2(1:2, i) - x1(1:2, i)).^2);
    end
    params.costs(plane.id, key{1}) = -50*distCost;
    params.costs(key{1}, plane.id) = -50*distCost;
end

% 
% for i = 1:N
%     for j = 1:numOtherPlanes
%         x2 = params.aircraft_list(j).x_yalmip;
%         pos2 = x2(1:2, :); %get xy position for other plane
%         distCost = sum((pos2-pos1).^2); %distance squared
%         params.obj = params.obj - 100*distCost;
%     end
% end