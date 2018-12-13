function params = addPlane(plane,params,N )

removePlane(params, plane.id); % First remove the plane in case it's already there

otherPlanes = keys(params.aircraft_list);

params.aircraft_list(plane.id) = plane.setup_yalmip(N,NaN);
%adds to aircraft array

params.color_list(plane.id) = rand(1,3);

params.costs(plane.id, plane.id) = plane.yalmip_cost;

%adds plane distance cost over N timesteps (PQR costs)

x1 = plane.x_yalmip;
for key = otherPlanes
    x2 = params.aircraft_list(key{1}).x_yalmip;
    for i = 1:(N+1) % x is size N + 1;
        distCost(i) = sum((x2(1:2, i) - x1(1:2, i)).^2);
    end
    % We actually only need to save it in one location. As long as both are
    % checked and both are deleted when the plane is removed, it should
    % work fine
    params.costs(plane.id, key{1}) = 0;
%     params.costs(key{1}, plane.id) = -50*distCost;
    cons = [];
    for k = 1:(N+1)
        cons = [cons, [distCost(k) >= 500]:[ ...
            plane.id, ' ', key{1}, ' collision constraints']];
    end
    params.collision_constraints(plane.id, key{1}) = cons;
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