function out = plotStateAndInputs(params)
numPlanes = numel(params.aircraft_list);
figure(2); clf
for i = 1:numPlanes
    plane = params.aircraft_list(i);
    stateArr = zeros(plane.nx,params.Ng);
    inputArr = zeros(plane.nu,params.Ng);
    for j = 1:params.Ng
        [state,input] = plane.getState(j);
        stateArr(:,j) = state;
        inputArr(:,j) = input;
    end
    numPlots = plane.nx+plane.nu;
    names = {'x','y','speed','heading angle'};
    for ii = 1:plane.nx
        subplot(numPlots,1,ii);
        plot(stateArr(ii,:),'+-','DisplayName',num2str(i));
        hold on
        title(names{ii});
        legend('-DynamicLegend');
    end
    inputnames = {'forward acceleration','bank angle'};
    for ii = 1:plane.nu
        subplot(numPlots,1,ii+plane.nx);
        plot(inputArr(ii,:),'+-','DisplayName',num2str(i))
        hold on
        title(inputnames{ii});
        legend('-DynamicLegend');
    end
    grid
end