%
clear

wid = 'Simulink:blocks:TDelayBufferTooSmall';
warning('off',wid)

%{
load_system('platoon_blocks')
print('-splatoon_blocks','-dpdf','platoon_blocks')
load_system('platoon_blocks/Leader')
print('-sLeader','-dpdf','Leader')
load_system('platoon_blocks/Vehicle 3')
print('-sVehicle 3','-dpdf','Vehicle_3')
load_system('platoon_blocks/Vehicle 3/Controller 3')
print('-sController 3','-dpdf','Controller_3')
load_system('platoon_blocks/Vehicle 3/Plant 3')
print('-sPlant 3','-dpdf','Plant_3')
%}


% front matter
% load inputs
for m=3%1:6
    %method = 'PL2';
    % define controller gains here
    switch m %method
        case 1 %'PLF1'
            leadervel = 2.7;
            leaderacc = 0.3;
            precacc = 0.2;
            precdsp = 3.6;
            precvel = 0.9;
            ti = 'Platoon Dynamics : Predecessor-Leader-Following w/ Accel';
            outfile = 'plf1_hills';
        case 3 %'PLF2'
            leadervel = 2.4;
            leaderacc = 0;
            precacc = 0;
            precdsp = 5.6;
            precvel = 1.9;
            ti = 'Platoon Dynamics : Predecessor-Leader-Following no Accel';
            outfile = 'plf2_hills';
        case 2 %'PL1'
            leadervel = 0;
            leaderacc = 0;
            precacc = 0.2;
            precdsp = 3.6;
            precvel = 0.9;
            ti = 'Platoon Dynamics : Predecessor-Following no Accel';
            outfile = 'pl1_hills';
        case 4 %'PL2'
            leadervel = 0;
            leaderacc = 0;
            precacc = 0.1;
            precdsp = 5.6;
            precvel = 1.7;
            ti = 'Platoon Dynamics : Predecessor-Following Higher Gains';
            outfile = 'pl2_hills';
        case 5 %'PLF1'
            leadervel = 4.7;
            leaderacc = 0.8;
            precacc = 0.2;
            precdsp = 3.6;
            precvel = 0.9;
            ti = 'Platoon Dynamics : Predecessor-Leader-Following w/ Accel';
            outfile = 'plf3_hills';
        case 6 %'PLF1'
            leadervel = 0;
            leaderacc = 0;
            precacc = 0.2;
            precdsp = 4.5;
            precvel = 1.;
            ti = 'Platoon Dynamics : Predecessor-Following';
            outfile = 'pl3_hills';
    end

    for v=2:6
        eval(['cvf1_' num2str(v) ' = ' num2str(leadervel) ';'])
        eval(['caf1_' num2str(v) ' = ' num2str(leaderacc) ';'])
        eval(['ca1_' num2str(v) ' = ' num2str(precacc) ';'])
        eval(['cv1_' num2str(v) ' = ' num2str(precvel) ';'])
        eval(['cx1_' num2str(v) ' = ' num2str(precdsp) ';'])
    end
    
    % hill
    x_hill = [-20:2000]; % breakpoints
    theta_hill = zeros(size(x_hill)); % just to linear
    
    % reference velocity
    t_vel = [0 6  25 28 35 40 50]';
    v_ref = [0 24 24 33 33 13 13]';

    sim('platoon_blocks_hills')
    platoon_blocks_hills
    print('-splatoon_blocks_hills','-dpdf','platoon_blocks_hills')

    % initialize some things
    tstart = 0;
    intT30 = find(V1out.Time>=tstart);
    time = V1out.Time(intT30)-tstart;
    a = 3; % column number of acceleration
    v = 1;
    x = 2;
    thrtl = 4;
    num_veh = 6;

    % velocity, accel, disp
    clear vel acc dsp delvel deldsp trt
    leg1 = cellstr('Leader');
    leg2 = cellstr('Leader-V1');
    for i=1:num_veh
        vel(:,i) = eval(['V' num2str(i) 'out.Data(intT30,v)']);
        acc(:,i) = eval(['V' num2str(i) 'out.Data(intT30,a)']);
        dsp(:,i) = eval(['V' num2str(i) 'out.Data(intT30,x)']);
        trt(:,i) = eval(['V' num2str(i) 'out.Data(intT30,thrtl)']);
    end

    % deltas
    for i=1:num_veh-1
        delvel(:,i) = eval(['V' num2str(i+1) 'out.Data(intT30,v)'])-...
            eval(['V' num2str(i) 'out.Data(intT30,v)']);
        deldsp(:,i) = eval(['V' num2str(i+1) 'out.Data(intT30,x)'])-...
            eval(['V' num2str(i) 'out.Data(intT30,x)']);
        leg1 = [leg1,cellstr(['System ' num2str(i)])];
        leg2 = [leg2,cellstr(['Vehicle ' num2str(i) '-' num2str(i+1)])];
    end
    leg2 = leg2(1:end-1);

    % plot
    f = figure(1);
    %set(f,'units','normalized','position',[0 0 .5 1]);
    f.PaperUnits = 'normalized';
    f.PaperPosition = [0 0 1 1];


    % velocities
    subplot(411)
    plot(time,VRef.Data(intT30,1),'--k','LineWidth',0.8)
    grid on
    hold on
    plot(time,vel,'LineWidth',1)
    hold off
    ylabel('Velocities [\it{m/s}\rm{]}','Interpreter','Latex')
    l=legend([cellstr('Ref'),leg1],'Location','SouthEast');
    set(l,'Interpreter','Latex')
    legend boxoff
    title(ti,'Interpreter','Latex')
    grid on, set(gca,'FontSize',14)

    % acceleration
    for p = 2:4
        subplot(4,1,p)
        switch p
            case 2
                data = acc;
                ylab = 'Acceleration [$m/s^{2}$]';
            case 3
                data = delvel;
                ylab = '$\Delta$ Velocity [\it{m/s}\rm{]}';
            case 4
                data = deldsp;
                ylab = '$\Delta$ Spacing [\it{m}\rm{]}';
        end
        plot(time,data,'LineWidth',1)
        grid on
        ylabel(ylab,'Interpreter','Latex')
        grid on, set(gca,'FontSize',14)

        % velocity difference
        %subplot(413)
        %plot(time,delvel,'LineWidth',2)
        %grid on
        %ylabel(,'Interpreter','Latex')
        if p==3
            l=legend(leg2,'Location','SouthEast');
            set(l,'Interpreter','Latex')
            legend boxoff
        end
        %grid on, %set(gca,'FontSize',14)

        % spacing difference
        %subplot(414)
        %plot(time,deldsp,'LineWidth',2)
        %grid on
        %xlabel('Time \it{t} \rm{[s]}','Interpreter','Latex'), 
        %ylabel(,'Interpreter','Latex')
        %grid on, %set(gca,'FontSize',14)
        %set(gca,'Position',[0.08 0.08 .84 .84])
    end
    xlabel('Time \it{t} \rm{[s]}','Interpreter','Latex')

    %saveas(gcf,['platoon_' outfile],'png')
    print(f,['platoon_' outfile],'-dpdf','-r0')
    save(['platoon_' outfile],'time','dsp','deldsp','vel','trt','VRef','leg1');
end


