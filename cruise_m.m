clear
close all

%%
%---- constants given in A&M-----
g = 9.81; % m/s^2
Cr = 0.01;
rho = 1.3; % kg/m^3
Cv = 1;
A = 2.4; % m2, frontal area of car
Cd = 0.32;

%
b = 1.32; % from (5.31) of A&M
ve = 20;
%m = alphan*Te/b;
masses = [1000,2000,3000];
m = 1600;
%m = 1670;
%m = masses(2);

%----- controller gains-----
% u = kp*dz/dt + ki*z = kp*e + ki*int(e(T))dT
% dz/dt = vr - v
kp = 0.5;
ki = 0.1;

%---- initialize time series
dt = 0.01; et = 30;
nt = et/dt + 1;
t = linspace(0,et,nt);
thetamax = 4*pi/180;

for i = 1:length(t)
    if t(i)<=5
        theta(i) = 0;
    elseif t(i)>=6
        theta(i) = thetamax;
    else
        theta(i) = (i - (5/dt+1))*dt*thetamax;
    end
end
%--- try just zero for theta
%theta = zeros(1,length(t));

%%
%---- torque for different gears ----
%alphan = 40, 25, 16, 12, 10
% 1/alphan -> effective wheel radius
% T = Tm(1-beta*(w/wn-1)^2)
% omega = n/r*v =: alphan*v
%Tm = 190; % max torque, Nm
%omegan = 420; % rad/s, given
%beta = 0.4; % given
v = linspace(0,70);
for i=1:5
    switch i
        case 1
            an = 40;
        case 2
            an = 25;
        case 3
            an = 16;
        case 4
            an = 12;
        case 5
            an = 10;
    end
    T(i,:) = throttle(an,v);
end
figure
plot(v,T,'LineWidth',2), grid on
xlabel('Velocity \it{v} \rm{[m/s]}'), ylabel('Torque \it{T} \rm{[Nm]}')
legend('n=1','n=2','n=3','n=4','n=5')
axis([0 v(end) 100 200])
set(gca,'FontSize',14)
saveas(gcf,'throttle','png')

%pick a gear
alphan = 25;
alphan = 12;
%Te = throttle(alphan,ve);


% initialize controller state (from running steady state at theta=0)
%z = 1.169880953797643; 
%z = 0.762310322653803;

% initialize things
z = 0;
vr = 20; % reference velocity
v = 20; % initial state, assume initial steady st
% initialize time series for velocity
dv = 1; %vel(1) = v;

%%
% find the equilibrium (starting point)
%i = 1;
while abs(dv)>0.00001
    % torque
    T = throttle(alphan,v);
    % control input based on ref vel
    dz = vr - v;   z = z + dz*dt;
    u = kp*dz + ki*z;
    % compute next time step
    dv = 1/m*alphan*u*T - g*Cr*sign(v) + ...
        - 1/2/m*rho*Cd*A*v^2 - g*sin(0);
    v = v + dv*dt; i = i + 1;
end
clear v dv dz u T
disp(['Initial state of (nonlinear) control variable, z=', num2str(z)])

%%
% initialize things for nl simulation
v = 20; % initial state, assume initial steady st

%run the nonlinear sim
for k = 1:length(t)
    %for i = 1:250
    % store v
    velnl(k) = v;
    
    % angle of road
    th = theta(k);
    
    % torque
    T = throttle(alphan,v);
    %Tm*(1-beta*(alphan*v/omegan-1).^2);
    
    % control input based on ref vel
    dz = vr - v;
    z = z + dz*dt;
    %z = dz*dt;
    u = kp*dz + ki*z;
    throtnl(k) = u;
    
    % compute next time step
    dv = 1/m*alphan*u*T - g*Cr*sign(v) + ...
        - 1/2/m*rho*Cd*A*v^2 - g*sin(th);
    v = v + dv*dt;
end

%figure
%subplot(211)
%plot([0 et],[ve,ve],'k',[5,5],[18,20.5],'--k',...
%    t(1:length(velnl)),velnl,'LineWidth',2)
%xlabel('Time \it{t} \rm{[s]}'), ylabel('Velocity \it{v} \rm{[m/s]}')
%grid on
%set(gca,'FontSize',14)
%subplot(212)
%plot([5,5],[0,1],'--k',...
%    t(1:length(throtnl)),throtnl,'LineWidth',2)
%xlabel('Time \it{t} \rm{[s]}'), ylabel('Throttle \it{u}')
%grid on
%set(gca,'FontSize',14)
%saveas(gcf,'nl_cruise','png')

%%
%------linearization-----
%alphan = 12; % 4th gear
ve = 20; % equil'm velocity, m/s
thetae = 0;

% calculate our own ue
ueE = 1/(alphan*throttle(alphan,ve))*(m*g*Cr*sign(ve)+1/2*rho*Cv*A*ve^2+m*g*sin(thetae));
%m1 = (ue*alphan^2)/a; %m2 = alphan*throttle(alphan,ve)/b;
%
aE = 1/m*(ueE*alphan*dTdv(alphan,ve) - rho*Cv*A*ve);
bE = alphan*throttle(alphan,ve)/m;
bg = g*cos(thetae);

fE = 1/m*alphan*ueE*throttle(alphan,ve) - g*Cr*sign(ve) - 1/2/m*rho*Cv*A*ve^2 +...
    -g*sin(thetae);

%----- equil'm from A&M -----, which I can't figure out
a = -0.0101; b = 1.32; ue = 0.1687; 
ve = 20;



%%
% initialize simulation 
vr = 20; % reference velocity
% find the equilibrium (starting point)
%i = 1;
dv = 1; z = 0;
while abs(dv)>0.00001
    % control input based on ref vel
    dz = vr - v;   z = z + dz*dt;
    u = kp*dz + ki*z;
    % compute next time step
    %dv = a*(v-ve) - bg*(theta(k)-thetae) + b*(u-ue);
    dv = a*(v-ve) + b*(u-ue);
    v = v + dv*dt; i = i + 1;
end
zlin = 0; dv = 1;
while abs(dv)>0.00001
    % control input based on ref vel
    dz = vr - v;   zlin = zlin + dz*dt;
    u = kp*dz + ki*zlin;
    % compute next time step
    %dv = a*(v-ve) - bg*(theta(k)-thetae) + b*(u-ue);
    %dv = a*(v-ve) + b*(u-ue);
    dv = fE + aE*(v-ve) + bE*(u-ue);
    v = v + dv*dt; i = i + 1;
end
clear v dv dz u
disp(['Initial state of (linear) control variable, z=', num2str(z)])
disp(['Initial state of (Taylor) control variable, z=', num2str(zlin)])

v = 20; % initial state, assume initial steady st
% run the linear sim
for k = 1:length(t)
    % store v
    vellin(k) = v;
    
    % control input based on ref vel
    dz = vr - v;
    z = z + dz*dt;
    u = kp*dz + ki*z;
    throtlin(k) = u;
        
    % compute next time step
    dv = a*(v-ve) - bg*(theta(k)-thetae) + b*(u-ue);
    v = v + dv*dt;
end

% run sim with Taylor series expansion
z = zlin;
for k = 1:length(t)
    % store v
    velTay(k) = v;
    
    % control input based on ref vel
    dz = vr - v;
    z = z + dz*dt;
    u = kp*dz + ki*z;
    throtTay(k) = u;
        
    % compute next time step
    dv = fE + aE*(v-ve) - bg*(theta(k)-thetae) + bE*(u-ue);
    v = v + dv*dt;
end

%%
% plot results
figure
subplot(211)
plot(t(1:length(velnl)),velnl,'LineWidth',2)
hold on %,...
plot(t(1:length(vellin)),vellin,'-.r','LineWidth',2)
plot(t(1:length(velTay)),velTay,'--m','LineWidth',2)
xlabel('Time \it{t} \rm{[s]}'), ylabel('Velocity \it{v} \rm{[m/s]}')
grid on, set(gca,'FontSize',14)
legend('Nonlinear','A&M','Taylor','Location','SouthEast')
legend boxoff
%hold on
plot([0 et],[vr,vr],'k',[5,5],[18.5,20.5],'--k','LineWidth',2)
hold off
subplot(212)
plot([5,5],[0,1],'--k',...
    t(1:length(throtnl)),throtnl,'LineWidth',2)
hold on
plot(t(1:length(throtlin)),throtlin,'-.r','LineWidth',2)
plot(t(1:length(throtTay)),throtTay,'--m','LineWidth',2)
xlabel('Time \it{t} \rm{[s]}'), ylabel('Throttle \it{u}')
grid on, set(gca,'FontSize',14)
hold off
%saveas(gcf,'nl_lin_cruise_mynumbers','png')

%%
% do some checks on A&M's derivation
check1 = dvdt(alphan,ve,ue,0);
check2 = dvdt(alphan,ve,ueE,0);
disp(['How accurate is AMs ue? ', num2str(check1)])
disp(['How accurate is my ue? ', num2str(check2)])
disp('closer to zero is better')

%%
% discrete time simulation

%% save data
save('script_sim_data','vellin','velnl','throtlin','throtnl','t','vr')