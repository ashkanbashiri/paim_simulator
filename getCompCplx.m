n_schedules = ones(1,50);
for i=1:50
    n_schedules(i) = compcplx(i);
    fprintf('%d: %d\n',i,n_schedules(i));
end
% p = polyfit(1:50,n_schedules,6);
f = fit([1:50]',n_schedules','exp1')
plot(f,[1:50]',n_schedules')
