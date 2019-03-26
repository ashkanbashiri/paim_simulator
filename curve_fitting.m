% x = load('ave_delays.mat');
% delays = x.AverageDelayPerVehicle
% delays = delays(3,:);
% save('delays.mat','delays');
flows = load('trafficFlows.mat');
flows = flows.trafficFlows;
delays = load('delays.mat');
delays = delays.delays
