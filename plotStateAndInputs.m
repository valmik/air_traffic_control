function out = plotStateAndInputs(params)
figure(2); clf

keySet = keys(params.aircraft_list);

for key = keySet
    plane = params.aircraft_list(key{1});
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
        plot(stateArr(ii,:),'+-', 'color', params.color_list(key{1}));
        hold on
    end
    inputnames = {'forward acceleration','bank angle'};
    for ii = 1:plane.nu
        subplot(numPlots,1,ii+plane.nx);
        plot(inputArr(ii,:),'+-', 'color', params.color_list(key{1}))
        hold on
    end
    grid
end

for ii = 1:plane.nx
    subplot(numPlots,1,ii);
    hold on
    title(names{ii});
    legend(keySet);
end

for ii = 1:plane.nu
    subplot(numPlots,1,ii+plane.nx);
    hold on
    title(inputnames{ii});
    legend(keySet);
end