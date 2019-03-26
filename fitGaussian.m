d = load('alldelays.mat');
d = d.delays;
size(d);
%h = histogram(d,100);
hist_fit = histfit(d,100)