v1 = load('v_history_1.mat');
v1 = v1.v_history;
v2 = load('v_history_2.mat');
v2 = v2.v_history;
v3 = load('v_history_3.mat');
v3 = v3.v_history;
v4 = load('v_history_4.mat');
v4 = v4.v_history;
v5 = load('v_history_5.mat');
v5 = v5.v_history;

br1 = load('br_history_1.mat');
br1 = br1.br_history;
br2 = load('br_history_2.mat');
br2 = br2.br_history;
br3 = load('br_history_3.mat');
br3 = br3.br_history;
br4 = load('br_history_4.mat');
br4 = br4.br_history;
br5 = load('br_history_5.mat');
br5 = br5.br_history;

gr1 = load('gear_history_1.mat');
gr1 = gr1.gear_history;
gr2 = load('gear_history_2.mat');
gr2 = gr2.gear_history;
gr3 = load('gear_history_3.mat');
gr3 = gr3.gear_history;
gr4 = load('gear_history_4.mat');
gr4 = gr4.gear_history;
gr5 = load('gear_history_5.mat');
gr5 = gr5.gear_history;



u1 = load('u_history_1.mat');
u1 = u1.u_history;
u2 = load('u_history_2.mat');
u2 = u2.u_history;
u3 = load('u_history_3.mat');
u3 = u3.u_history;
u4 = load('u_history_4.mat');
u4 = u4.u_history;
u5 = load('u_history_5.mat');
u5 = u5.u_history;

dx1 = load('dx_history_1.mat');
dx1 = dx1.dx_history;
dx2 = load('dx_history_2.mat');
dx2 = dx2.dx_history;
dx3 = load('dx_history_3.mat');
dx3 = dx3.dx_history;
dx4 = load('dx_history_4.mat');
dx4 = dx4.dx_history;
dx5 = load('dx_history_5.mat');
dx5 = dx5.dx_history;


totalTime = length(v1);
times = 1:totalTime;
times = times/10;
l = length(times);
plot(times,v1,'LineWidth',2);
hold on
plot(times,v2,'LineWidth',2);
hold on
plot(times,v3,'LineWidth',2);
hold on
plot(times,v4,'LineWidth',2);
hold on
plot(times,v5,'LineWidth',2);
legend('leader','follower-1','follower-2','follower-3','follower-4');
ylabel('Speed (m/s)');
xlabel('time (s)');
title('Speed');
grid minor
%set(gca,'Color','y');


%plot leader and followers throttle and brake input
figure
subplot(3,2,1);
plot(times,u2,'g','LineWidth',2);
hold on
plot(times,br2,'LineWidth',2,'Color','r');
hold on
plot(times,gr2,'LineWidth',2);
ylabel('Throttle-Brake-Gear');
xlabel('time (s)');
title('Follower1 Throttle-Brake-Gear');
legend('Throttle','Brake','Gear');
%set(gca,'Color',[0.5 0.5 0.5],'LineWidth',5);
grid minor

subplot(3,2,2);
plot(times,u3,'g','LineWidth',2);
hold on
plot(times,br3,'LineWidth',2,'Color','r');
hold on
plot(times,gr3,'LineWidth',2);
ylabel('Throttle-Brake-Gear');
xlabel('time (s)');
title('Follower2 Throttle-Brake-Gear');
legend('Throttle','Brake','Gear');
%set(gca,'Color',[0.5 0.5 0.5],'LineWidth',5);
grid minor

subplot(3,2,3);
plot(times,u4,'g','LineWidth',2);
hold on
plot(times,br4,'LineWidth',2,'Color','r');
hold on
plot(times,gr4,'LineWidth',2);
ylabel('Throttle-Brake-Gear');
xlabel('time (s)');
title('Follower3 Throttle-Brake-Gear');
legend('Throttle','Brake','Gear');
%set(gca,'Color',[0.5 0.5 0.5],'LineWidth',5);
grid minor

subplot(3,2,4);
plot(times,u5,'g','LineWidth',2);
hold on
plot(times,br5,'LineWidth',2,'Color','r');
hold on
plot(times,gr5,'LineWidth',2);
ylabel('Throttle-Brake-Gear');
xlabel('time (s)');
title('Follower4 Throttle-Brake-Gear');
legend('Throttle','Brake','Gear');
%set(gca,'Color',[0.5 0.5 0.5],'LineWidth',5);
grid minor

subplot(3,2,5:6);
plot(times,u1,'g','LineWidth',2);
hold on
plot(times,br1,'LineWidth',2,'Color','r');
hold on
plot(times,gr5,'LineWidth',2,'Color','k');
ylabel('Throttle-Brake-Gear');
xlabel('time (s)');
title('Leader Throttle-Brake-Gear');
legend('Throttle','Brake','Gear');

%set(gca,'Color',[0.5 0.5 0.5],'LineWidth',5);
grid minor





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% plot(times,u1);
% hold on
% figure
% plot(times,u1);
% hold on;
% plot(times,u2);
% hold on
% plot(times,u3);
% hold on
% plot(times,u4);
% hold on
% plot(times,u5);
% hold on
% plot(times,br1);
% hold on;
% plot(times,br2);
% hold on
% plot(times,br3);
% hold on
% plot(times,br4);
% hold on
% plot(times,br5);
% legend('leader-Throttle','follower1-Throttle','follower2-Throttle','follower3-Throttle','follower4-Throttle');
% legend('leader-Brake','follower1-Brake','follower2-Brake','follower3-Brake','follower4-Brake');
% ylabel('Input Throttle and Brake');
% xlabel('time (s)');
% title('Input Throttle and Brake');
% 
% grid minor

figure
% plot(times,dx1);
% hold on
plot(times,dx2);
hold on
plot(times,dx3);
hold on
plot(times,dx4);
hold on
plot(times,dx5);
legend('follower-1','follower-2','follower-3','follower-4');
ylabel('Distance To Preceding (m)');
xlabel('time (s)');
title('Distance to Preceding Vehicle');
grid minor

figure
plot(times,gr1,'LineWidth',2);
hold on
plot(times,gr2,'LineWidth',2);
hold on
plot(times,gr3,'LineWidth',2);
hold on
plot(times,gr4,'LineWidth',2);
hold on
plot(times,gr5,'LineWidth',2);
legend('leader','follower-1','follower-2','follower-3','follower-4');
ylabel('Gear Position');
xlabel('time (s)');
title('Gear Position');
grid minor
%set(gca,'Color','y');
