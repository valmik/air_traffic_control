function [out] = plotStateAndInputs_test(timeArr,statesArr)

% NOTE: timeArr should have N-1 timesteps while states should be of size N
% timeArr (N-1 timesteps x 1)
% statesArr (4 states x N timesteps)
% 1st state = x
% 2nd state = y
% 3rd state = speed
% 4th state = heading angle

% Test data/script
% timeArr = [0.1,0.5,0.3,0.7];
% xs = [-3000,-1100,100,2333,5000];
% ys = [-5000,-3200,-9,1421,4321];
% speeds = [200,275,315,343,472];
% headingAngles = [0,5,10,15,20];
% statesArr = [xs;ys;speeds;headingAngles];
% out = plotStateAndInputs_test(timeArr,statesArr);

names = {'x','y','speed','heading angle'};
N = numel(timeArr)+1;

% FIRST: let's plot what the states currently look like
% figure();
% idx = 1:1:N;
% for i = 1:4
%     subplot(4,1,i);
%     plot(idx,statesArr(i,:),'-o');
%     title(names{i});
% end

times = zeros(N,1);
times(1) = 0;
for i = 1:numel(timeArr)
    times(i+1) = times(i) + timeArr(i);
end
% BUT, this is what they should really look like
% figure();
% subplot(4,1,4);
% for i = 1:4
%     subplot(4,1,i);
%     plot(times,statesArr(i,:),'-o');
%     title(names{i});
% end

% % NOW: we'll "resample", save the data in 'out', and plot
minTS = min(timeArr);
timeQ = times(1):minTS:times(end);
%[num, den] = rat(minTS);
out = zeros(4,numel(timeQ));
for i = 1:4
    %out(i,:) = resample(statesArr(i,:),den,num);
    out(i,:) = interpn(times,statesArr(i,:),timeQ);
end
figure();
subplot(4,1,4);
for i = 1:4
    subplot(4,1,i);
    plot(timeQ,out(i,:),'-o');
    title(names{i});
end


end