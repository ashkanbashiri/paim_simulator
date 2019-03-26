classdef vehicleModel < handle
    
    properties(SetAccess = public)
        %Dt Time step for simulation of the robot
        fuelConsumed = 0;
        pNumber;
        seeInfo=0;
        counter = 1;
        u_history = [];
        v_history = [];
        dx_history = [];
        br_history = [];
        gear_history = [];
        theta = 0;
        w_ei=0;
        prev_dtheta = 0;
        
        delta = 0;%Angle of the front wheels
        b_wheels = 1.5;%distance between front and rear wheels
        %w = (v*tan(delta)/b)   Angular velocity of the vehicle
        
        crossing=0;
        mynumber;
        w = 0;
        Dt = 1;
        HeadingVectorLength = 1.5;
        leaderVelocity = 1;
        desiredDistanceToPreceding = 10;
        CX = 3.6;
        CVE = 0.9;
        CV = 2.4;
        CA = 0;
        CAF = 0;
        TAU = 0.1;
        velocity = 1;
        acceleration = 0;
        dthetai = 0;
        kiw = 0.0;
        kdw = 0.0;
        kpw = 0.01;%angular velocity error gain
        u = 0;
        
        brake_friction = .9;
        kp = .35;%3.5;%linear velocity to preceding error gain
        kpx = .75;%1.5;%headway error gain
        kplv = 0.35;%3.5;%linear velocity to leader error gain
        
        
        kppa = 0.0;%linear acceleration to preceding error gain
        ki = 0.1;
        z = 0;
        as = [40 25 26 12 10];%alpha for gears 1 through 5
        gear = 1;
        %---- constants given in A&M-----
        g = 9.81; % m/s^2
        Cr = 0.01;
        rho = 1.3; % kg/m^3
        Cv = 1;
        A = 2.4; % m2, frontal area of car
        Cd = 0.32;
        alphan = 40;%Gear 4
        m = 1600;
        
        %
        
        velocityOfPreceding = 1;
        distanceToPreceding = 2;
        showNumber;
        
        %CurrentPose Current pose of the robot
        CurrentPose;
        drawVehicle = 1;
        vel = 5;%initial velocity
        acc = 0;%initial acceleration
        role = 'leader';
        status = 'moveandwait';%'stopandwait','crossing','done'
        color = 'b';
        myState = struct('pose',[0 0 0],'vel',1,'acc',0);
        lane = 1;
        %Trajectory Pose history of the robot
        Trajectory = [];
    end
    
    properties(Access = private)
        %HeadingVectorLength Length of the heading vector for plotting
        %HRobot Graphics handle for robot
        HRobot;
        drawn;
        
        %HRobotHeading Graphics handle for robot heading
        HRobotHeading;
        
        %HRobotTrail Graphics handle for robot pose history
        HRobotTrail;
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %function cross for the leader crossing the intersection
        function [myx,myy,myv,mya] = cross(obj,dt,draw,color,px,py,pv,pa,lv,la)
            k_p = 1;
            k_i = 0.1;
            k_d = 0.1;
            k_p_w = 1;
            k_i_w = 0.1;
            k_d_w = 0.1;
            if(obj.mynumber>1)
                obj.kiw = 0.0;
                obj.kpw = 0.1;
                obj.kdw = 0.01;
                obj.desiredDistanceToPreceding = obj.vel*1 + 9;%four meters behind preceding at speed=0

%                 obj.kp = .35;%3.5;%linear velocity to preceding error gain
%                 obj.kpx = .35;%1.5;%headway error gain
%                 obj.kplv = 0.55;%3.5;%linear velocity to leader error gain
            else
                 obj.kpw = 0.2;
                 obj.kiw = 0.0;
                 obj.kdw = 0.03;
                 obj.kp = .35;%3.5;%linear velocity to preceding error gain
                 obj.kpx = .75;%1.5;%headway error gain
                 obj.kplv = 0.35;%3.5;%linear velocity to leader error gain
%                 obj.kpx = 0.95;
%                 obj.kp = .0;
%                 obj.kplv = 0.25;
            end
            if obj.crossing==0
                obj.desiredDistanceToPreceding = obj.vel*1 + 9;%four meters behind preceding at speed=0
            end
            cur_vel = 3.6 * obj.vel;
            if(obj.gear==1 && cur_vel>30)%Shifting from gear 1 to 2
                obj.alphan = obj.as(2);
                obj.gear = 2;
            elseif(obj.gear == 2 && cur_vel<30)%shifting from gear 2 to 1
                obj.alphan = obj.as(1);
                obj.gear = 1;
            elseif(obj.gear==2 && cur_vel>65)%shifting from gear 2 to 3
                obj.alphan = obj.as(3);
                obj.gear = 3;
            elseif(obj.gear==3 && cur_vel>100)%shifting from gear 3 to 4
                obj.alphan = obj.as(4);
                obj.gear = 4; 
            elseif(obj.gear==3 && cur_vel<55)%shifting from gear 3 to 2
                obj.alphan = obj.as(2);
                obj.gear = 2; 
            elseif(obj.gear==4 && cur_vel<90)%shifting from gear 4 to 3
                obj.alphan = obj.as(3);
                obj.gear = 3;
            end
                
                mya = obj.acc;
                myv = obj.vel;
                obj.color = color;
                curPose = obj.CurrentPose;
                myx = curPose(1);
                myy = curPose(2);
                if(myx>100)
                    if(myx>150)
                        obj.theta = 0;
                    else
                        obj.theta =0;% pi/9;
                    end
                end
                
                %compute v and w using cacc
                bearing = atan2(py-curPose(2),px-curPose(1));
                dtheta = atan2(sin(bearing-curPose(3)),cos(bearing-curPose(3)));
                obj.dthetai = (dtheta-obj.prev_dtheta)/dt;%obj.dthetai + dtheta*dt;%turned into kd
                obj.prev_dtheta = dtheta;
                obj.w_ei = obj.w_ei + dtheta*dt;
                obj.delta = dtheta *obj.kpw + obj.dthetai*obj.kdw ...
                    + obj.w_ei*obj.kiw;
                %             if(obj.mynumber==2)
                %                 fprintf('uw = %.2f, dtheta = %.2f\n',uw,dtheta);
                %             end
                %w = uw;
%                 if obj.mynumber==1 && obj.delta~=0
%                 fprintf('steering angle = %.2f\n',obj.delta);
%                 end
                obj.w = obj.vel*tan(obj.delta)/obj.b_wheels;
                dv = pv - obj.vel;
                % torque
                T = throttle(obj.alphan,obj.vel);
                if(obj.mynumber==1)
                    %fprintf('gear = %d , alphan = %d\n',obj.gear,obj.alphan);
                end
                % control input based on ref vel
                %dlv = lv-obj.vel;
                %dx = norm([px py] - curPose(1:2))-obj.desiredDistanceToPreceding;
                %dz = pv - obj.vel;   obj.z = obj.z + dz*dt;
                %u = obj.kp*dz + obj.kpx*dx + obj.kplv * dlv;
                dx = norm([px py]-[curPose(1) curPose(2)]);
                u = k_p * dx;
                temp = 0;
                if(u<0)
                    temp = -max(-1,u);
                    obj.Cr = obj.brake_friction*temp;
                    u = 0;
                else
                    u = min(u,1);
                    obj.Cr = 0.01;
                end
                % compute next time step
                obj.acc = 1/obj.m*obj.alphan*u*T - obj.g*obj.Cr*sign(obj.vel) + ...
                    - 1/2/obj.m*obj.rho*obj.Cd*obj.A*obj.vel^2 - obj.g*sin(obj.theta);
                v = obj.vel + obj.acc*dt;
                v = max(v,0);
                %v=1.1;
                %w=0;
                
                
                %end of computing v and w
                obj.acc = (v-obj.vel)/dt;
                obj.vel = v;
                x = curPose(1) + v*cos(curPose(3))*dt;
                y = curPose(2) + v*sin(curPose(3))*dt;
                theta = curPose(3) + obj.w*dt;
                obj.CurrentPose = [x, y, theta];
                
                %save u,v and dx
                obj.br_history(obj.counter) = temp;
                obj.gear_history(obj.counter) = obj.gear;
                obj.u_history(obj.counter) = u;
                obj.dx_history(obj.counter) = dx+obj.desiredDistanceToPreceding;
                obj.v_history(obj.counter) = obj.vel;
                obj.counter = obj.counter+1;
                if(draw==1 && ((curPose(1)>=0 && curPose(1)<=400 && curPose(2)>=0 && curPose(2)<=400) || curPose(1)==1000))
                    if(obj.drawn==0)
                        ax = gca;
                        hold(ax, 'on');
                        [x, y] = obj.getVehicleBody(obj.CurrentPose, obj.HeadingVectorLength);
                        % Draw robot
                        if(obj.drawVehicle==1)
                        obj.HRobot = plot(x,y, obj.color, 'Parent', ax);
                        end
                        % Draw heading vector
                        [xVec, yVec] = obj.computeHeadingVector(obj.CurrentPose, obj.HeadingVectorLength);
                        if(obj.drawVehicle==1)
                        obj.HRobotHeading = plot(ax, xVec, yVec, 'k', 'LineWidth', 3);
                        % Draw robot trail
                        %obj.HRobotTrail = plot(ax, robotCurrentLocation(1),  robotCurrentLocation(2), 'LineWidth', 2);
                        
                        hold(ax, 'off');
                        obj.drawn = 1;
                        end
                    end
                    updatePlots(obj);
                end
                
            end
        
        
        
        
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        
        function obj = vehicleModel(currentPose,lane,mynumber,pNumber)
            obj.pNumber = pNumber;
            obj.mynumber = mynumber;
            
            obj.CurrentPose = currentPose;
            obj.lane = lane;
            obj.role = 'leader';
            %robotCurrentLocation = obj.CurrentPose(1:2);
            
            %obj.Trajectory = obj.CurrentPose;
            if(currentPose(1)>=20 && currentPose(1)<=380 && currentPose(2)>=20 && currentPose(2)<=380)
                if(obj.drawVehicle==1)
                ax = gca;
                hold(ax, 'on');
                end
                
                [x, y] = obj.getVehicleBody(obj.CurrentPose, obj.HeadingVectorLength);
                % Draw robot
                obj.myState.pose = [x y 0];
                if(obj.drawVehicle==1)
                obj.HRobot = plot(x,y, obj.color, 'Parent', ax);
                end
                
                if(obj.mynumber==1 && obj.seeInfo==1)
                %obj.showNumber = text(obj.CurrentPose(1)+5*cos(obj.CurrentPose(3)+pi/2),obj.CurrentPose(2)+5*sin(obj.CurrentPose(3)+pi/2),sprintf('%.0fml',obj.fuelConsumed),'FontSize',8,'FontWeight','bold');
                obj.showNumber = text(obj.CurrentPose(1)+5*cos(obj.CurrentPose(3)+pi/2),obj.CurrentPose(2)+5*sin(obj.CurrentPose(3)+pi/2),sprintf('%d',obj.pNumber),'FontSize',8,'FontWeight','bold');

                end
                
                % Draw heading vector
                [xVec, yVec] = obj.computeHeadingVector(obj.CurrentPose, obj.HeadingVectorLength);
                if(obj.drawVehicle==1)
                obj.HRobotHeading = plot(ax, xVec, yVec, 'k', 'LineWidth', 3);
                
                
                % Draw robot trail
                %obj.HRobotTrail = plot(ax, robotCurrentLocation(1),  robotCurrentLocation(2), 'LineWidth', 2);
                
                hold(ax, 'off');
                obj.drawn = 1;
                end
            else
                obj.drawn = 0;
            end
        end
        function setRadius(obj,robotRadius)
            obj.HeadingVectorLength = robotRadius;
        end
        function [curx,cury,vel,acc] = driveLeader(obj,v,w,dt,draw,color)
            obj.v_history(obj.counter) = v;
            obj.counter = obj.counter+1;
            obj.w = w;
            obj.color = color;
            acc = obj.acc;
            vel = obj.vel;
            obj.acc = (v-obj.vel)/dt;
            obj.vel = v;
            curPose = obj.CurrentPose;
            curx = curPose(1);
            cury = curPose(2);
            x = curPose(1) + v*cos(curPose(3))*dt;
            y = curPose(2) + v*sin(curPose(3))*dt;
            theta = curPose(3) + w*dt;
            obj.CurrentPose = [x, y, theta];
            obj.myState.pose = obj.CurrentPose;
            obj.myState.vel = 1;
            obj.myState.acc = 0;
            if(draw==1 && ((curPose(1)>=0 && curPose(1)<=400 && curPose(2)>=0 && curPose(2)<=400) || curPose(1)==1000))
                if(obj.drawn==0)
                    ax = gca;
                    hold(ax, 'on');
                    [x, y] = obj.getVehicleBody(obj.CurrentPose, obj.HeadingVectorLength);
                    % Draw robot
                                    if(obj.drawVehicle==1)

                    obj.HRobot = plot(x,y, obj.color, 'Parent', ax);
                                    end
                    % Draw heading vector
                    [xVec, yVec] = obj.computeHeadingVector(obj.CurrentPose, obj.HeadingVectorLength);
                                    if(obj.drawVehicle==1)

                    obj.HRobotHeading = plot(ax, xVec, yVec, 'k', 'LineWidth', 3);
                    % Draw robot trail
                    %obj.HRobotTrail = plot(ax, robotCurrentLocation(1),  robotCurrentLocation(2), 'LineWidth', 2);
                    
                    hold(ax, 'off');
                    obj.drawn = 1;
                                    end
                end
                updatePlots(obj);
            end
            
        end
        
        function saveData(obj)
            u_history = obj.u_history;
            dx_history = obj.dx_history;
            v_history = obj.v_history;
            br_history = obj.br_history;
            gear_history = obj.gear_history;
            u_filename = strcat('u_history_',num2str(obj.mynumber));
            v_filename = strcat('v_history_',num2str(obj.mynumber));
            dx_filename = strcat('dx_history_',num2str(obj.mynumber));
            br_filename = strcat('br_history_',num2str(obj.mynumber));
            gear_filename = strcat('gear_history_',num2str(obj.mynumber));
            save(u_filename,'u_history');
            save(dx_filename,'dx_history');
            save(v_filename,'v_history');
            save(br_filename,'br_history');
            save(gear_filename,'gear_history');
        end
        
        function [myx,myy,myv,mya] = drive2(obj,dt,draw,color,px,py,pv,pa,lv,la,status)
            obj.status = status;
            if(obj.mynumber>1)
                obj.kiw = 0.0;
                obj.kpw = 0.27;
                obj.kdw = 0.03;
                obj.desiredDistanceToPreceding = obj.vel*1 + 9;%four meters behind preceding at speed=0

%                 obj.kp = .35;%3.5;%linear velocity to preceding error gain
%                 obj.kpx = .35;%1.5;%headway error gain
%                 obj.kplv = 0.55;%3.5;%linear velocity to leader error gain
            else
%                 lv = 0;
%                 pv = 0;
                 obj.kpw = 0.27;
                 obj.kiw = 0.0;
                 obj.kdw = 0.03;
                 obj.kp = .8;%3.5;%linear velocity to preceding error gain
                 obj.kpx = .75;%1.5;%headway error gain
                 obj.kplv = 0.0;%3.5;%linear velocity to leader error gain
%                 obj.kpx = 0.95;
%                 obj.kp = .0;
%                 obj.kplv = 0.25;
            end
            if obj.crossing==0
                obj.desiredDistanceToPreceding = obj.vel*1 + 9;%four meters behind preceding at speed=0
            end
            cur_vel = 3.6 * obj.vel;
            if(obj.gear==1 && cur_vel>30)%Shifting from gear 1 to 2
                obj.alphan = obj.as(2);
                obj.gear = 2;
            elseif(obj.gear == 2 && cur_vel<30)%shifting from gear 2 to 1
                obj.alphan = obj.as(1);
                obj.gear = 1;
            elseif(obj.gear==2 && cur_vel>65)%shifting from gear 2 to 3
                obj.alphan = obj.as(3);
                obj.gear = 3;
            elseif(obj.gear==3 && cur_vel>100)%shifting from gear 3 to 4
                obj.alphan = obj.as(4);
                obj.gear = 4; 
            elseif(obj.gear==3 && cur_vel<55)%shifting from gear 3 to 2
                obj.alphan = obj.as(2);
                obj.gear = 2; 
            elseif(obj.gear==4 && cur_vel<90)%shifting from gear 4 to 3
                obj.alphan = obj.as(3);
                obj.gear = 3;
            end
                
                mya = obj.acc;
                myv = obj.vel;
                obj.color = color;
                curPose = obj.CurrentPose;
                myx = curPose(1);
                myy = curPose(2);
                if(myx>100)
                    if(myx>150)
                        obj.theta = 0;
                    else
                        obj.theta =0;% pi/9;
                    end
                end
                
                %compute v and w using cacc
                bearing = atan2(py-curPose(2),px-curPose(1));
                dtheta = atan2(sin(bearing-curPose(3)),cos(bearing-curPose(3)));
                obj.dthetai = (dtheta-obj.prev_dtheta)/dt;%obj.dthetai + dtheta*dt;%turned into kd
                obj.prev_dtheta = dtheta;
                obj.w_ei = obj.w_ei + dtheta*dt;
                obj.delta = dtheta *obj.kpw + obj.dthetai*obj.kdw ...
                    + obj.w_ei*obj.kiw;
                %             if(obj.mynumber==2)
                %                 fprintf('uw = %.2f, dtheta = %.2f\n',uw,dtheta);
                %             end
                %w = uw;
%                 if obj.mynumber==1 && obj.delta~=0
%                 fprintf('steering angle = %.2f\n',obj.delta);
%                 end
                if(strcmp(obj.status,'stopandwait') || strcmp(obj.status,'moveandwait'))
                    obj.w = 0;
                else
                obj.w = obj.vel*tan(obj.delta)/obj.b_wheels;
                end
                dv = pv - obj.vel;
                % torque
                T = throttle(obj.alphan,obj.vel);
                if(obj.mynumber==1)
                    %fprintf('gear = %d , alphan = %d\n',obj.gear,obj.alphan);
                end
                % control input based on ref vel
                dlv = lv-obj.vel;
                dx = norm([px py] - curPose(1:2))-obj.desiredDistanceToPreceding;
                dpv = pv - obj.vel;   obj.z = obj.z + dpv*dt;
                u = obj.kp*dpv + obj.kpx*dx + obj.kplv * dlv;
                temp = 0;
                if(u<0)
                    temp = -max(-1,u);
                    obj.Cr = obj.brake_friction*temp;
                    u = 0;
                else
                    u = min(u,1);
                    obj.Cr = 0.01;
                end
                % compute next time step
                obj.acc = 1/obj.m*obj.alphan*u*T - obj.g*obj.Cr*sign(obj.vel) + ...
                    - 1/2/obj.m*obj.rho*obj.Cd*obj.A*obj.vel^2 - obj.g*sin(obj.theta);
                v = obj.vel + obj.acc*dt;
                
                v = max(v,0);
                obj.fuelConsumed = obj.fuelConsumed + fuelConsumption(v,obj.acc)*dt;%update fuel consumed
                %v=1.1;
                %w=0;
                
                
                %end of computing v and w
                obj.acc = (v-obj.vel)/dt;
                obj.vel = v;
                x = curPose(1) + v*cos(curPose(3))*dt;
                y = curPose(2) + v*sin(curPose(3))*dt;
                theta = curPose(3) + obj.w*dt;
                obj.CurrentPose = [x, y, theta];
                
                %save u,v and dx
                obj.br_history(obj.counter) = temp;
                obj.gear_history(obj.counter) = obj.gear;
                obj.u_history(obj.counter) = u;
                obj.dx_history(obj.counter) = dx+obj.desiredDistanceToPreceding;
                obj.v_history(obj.counter) = obj.vel;
                obj.counter = obj.counter+1;
                if(draw==1 && ((curPose(1)>=0 && curPose(1)<=400 && curPose(2)>=0 && curPose(2)<=400) || curPose(1)==1000))
                    if(obj.drawn==0)
                        ax = gca;
                        hold(ax, 'on');
                        [x, y] = obj.getVehicleBody(obj.CurrentPose, obj.HeadingVectorLength);
                        % Draw robot
                                        if(obj.drawVehicle==1)

                        obj.HRobot = plot(x,y, obj.color, 'Parent', ax);
                                        end
                        % Draw heading vector
                        [xVec, yVec] = obj.computeHeadingVector(obj.CurrentPose, obj.HeadingVectorLength);
                                        if(obj.drawVehicle==1)

                        obj.HRobotHeading = plot(ax, xVec, yVec, 'k', 'LineWidth', 3);
                        % Draw robot trail
                        %obj.HRobotTrail = plot(ax, robotCurrentLocation(1),  robotCurrentLocation(2), 'LineWidth', 2);
                        
                        hold(ax, 'off');
                        obj.drawn = 1;
                                        end
                    end
                    updatePlots(obj);
                end
                
            end
            function myState = drive(obj, v, w,dt,draw,color,precedingState,newState)
                %drive Drive the robot by integrating the kinematics
                %   drive(OBJ, V, W) updates the current pose of the robot by
                %   integrating the equations of the motion with linear
                %   velocity input V and angular velocity input W.
                curPose = obj.CurrentPose;
                obj.myState.pose = obj.CurrentPose;
                obj.color = color;
                obj.status = newState;
                myState = struct('pose',[0 0 0],'vel',0,'acc',0);
                myState.pose = obj.myState.pose;
                myState.vel = obj.myState.vel;
                myState.acc = obj.myState.acc;
                obj.TAU = dt*1;
                if(strcmp(obj.status,'moveandwait'))
                    v = 10;
                end
                if(~strcmp(obj.role,'leader')  &&  strcmp(obj.status,'moveandwait')) %Longitudinal Control: USE CACC for Followers
                    
                    %Compute velocity according using CACC
                    %whos(precedingState)
                    
                    vp = precedingState.vel;
                    ap = precedingState.acc;
                    xp = precedingState.pose(1);
                    vf = 10;
                    af = 0;
                    temp_x = myState.pose(1);
                    desiredDist = 14;
                    obj.u = obj.CX*(xp-temp_x-desiredDist) + obj.CV*(vf-obj.myState.vel) + ...
                        obj.CVE*(vp--obj.myState.vel);
                    %obj.u = 0;
                    %+ obj.CA *(ap - obj.myState.acc) + ...
                    %   obj.CAF*(af - obj.myState.acc);
                    %End of Computing velocity
                    obj.myState.acc =obj.myState.acc -obj.myState.acc + obj.u;
                    obj.myState.vel = obj.myState.vel + obj.myState.acc;
                    switch(obj.lane)
                        case 1
                            obj.myState.pose(1) = obj.myState.pose(1) + obj.myState.vel*obj.TAU;
                        case 2
                            obj.myState.pose(2) = obj.myState.pose(2) + obj.myState.vel*obj.TAU;
                        case 3
                            obj.myState.pose(1) = obj.myState.pose(1) - obj.myState.vel*obj.TAU;
                        case 4
                            obj.myState.pose(2) = obj.myState.pose(2) - obj.myState.vel*obj.TAU;
                    end
                    %x = curPose(1) + v*cos(curPose(3))*dt;
                    %y = curPose(2) + v*sin(curPose(3))*dt;
                    %theta = curPose(3) + w*dt;
                    %obj.CurrentPose = [x, y, theta];
                    pp = obj.myState.pose;
                    obj.CurrentPose = obj.myState.pose;
                    
                    
                else
                    
                    %For cases other than longitudinal control, use
                    %PurePursuit
                    x = curPose(1) + v*cos(curPose(3))*dt;
                    y = curPose(2) + v*sin(curPose(3))*dt;
                    theta = curPose(3) + w*dt;
                    obj.CurrentPose = [x, y, theta];
                    obj.myState.pose = obj.CurrentPose;
                    obj.myState.vel = 1;
                    obj.myState.acc = 0;
                    %obj.Trajectory = [obj.Trajectory; obj.CurrentPose];
                end
                if(draw==1 && ((curPose(1)>=0 && curPose(1)<=400 && curPose(2)>=0 && curPose(2)<=400) || curPose(1)==1000))
                    if(obj.drawn==0)
                        ax = gca;
                        hold(ax, 'on');
                        [x, y] = obj.getVehicleBody(obj.CurrentPose, obj.HeadingVectorLength);
                        % Draw robot
                                        if(obj.drawVehicle==1)

                        obj.HRobot = plot(x,y, obj.color, 'Parent', ax);
                                        end
                        % Draw heading vector
                        [xVec, yVec] = obj.computeHeadingVector(obj.CurrentPose, obj.HeadingVectorLength);
                                        if(obj.drawVehicle==1)

                        obj.HRobotHeading = plot(ax, xVec, yVec, 'k', 'LineWidth', 3);
                        % Draw robot trail
                        %obj.HRobotTrail = plot(ax, robotCurrentLocation(1),  robotCurrentLocation(2), 'LineWidth', 2);
                        
                        hold(ax, 'off');
                        obj.drawn = 1;
                                        end
                    end
                    updatePlots(obj);
                end
            end
        end
        
        
        methods(Access = private)
            function updatePlots(obj)
                %updatePlots Update all plots
                
                updateHeading(obj);
                updateRobotCurrentLocation(obj);
                %updateRobotTrail(obj);
                %drawnow;
            end
            function updateHeading(obj)
                %updateRobotCurrentLocation Update the X/YData for heading vector
                [xVec, yVec] = obj.computeHeadingVector(obj.CurrentPose, obj.HeadingVectorLength);
                obj.HRobotHeading.XData = xVec;
                obj.HRobotHeading.YData = yVec;
            end
            function updateRobotCurrentLocation(obj)
                %updateRobotCurrentLocation Update the X/YData for robot plot
                [robotXData, robotYData] = obj.getVehicleBody(obj.CurrentPose, obj.HeadingVectorLength);
                                if(obj.drawVehicle==1)

                obj.HRobot.Color = obj.color;
                obj.HRobot.XData =  robotXData;
                obj.HRobot.YData =  robotYData;
                                end
                if(obj.mynumber==1 && obj.seeInfo ==1)
                delete(obj.showNumber);
                %obj.showNumber = text(obj.CurrentPose(1)+5*cos(obj.CurrentPose(3)+pi/2),obj.CurrentPose(2)+5*sin(obj.CurrentPose(3)+pi/2),sprintf('%.0fml',obj.fuelConsumed),'FontSize',8,'FontWeight','bold');
                obj.showNumber = text(obj.CurrentPose(1)+5*cos(obj.CurrentPose(3)+pi/2),obj.CurrentPose(2)+5*sin(obj.CurrentPose(3)+pi/2),sprintf('%d',obj.pNumber),'FontSize',8,'FontWeight','bold');

                end
            end
            function updateRobotTrail(obj)
                %updateRobotTrail Update the X/YData for pose history
                robotCurrentLocation = obj.CurrentPose(1:2);
                obj.HRobotTrail.XData = [obj.HRobotTrail.XData  robotCurrentLocation(1)];
                obj.HRobotTrail.YData = [obj.HRobotTrail.YData  robotCurrentLocation(2)];
            end
        end
        
        methods(Access = private, Static)
            function [xVec, yVec] = computeHeadingVector(robotCurrentPose, robotDiameter)
                %computeHeadingVector Compute XData and YData for heading vector
                
                robotCurrentLocation = robotCurrentPose(1:2);
                cth = cos(robotCurrentPose(3));
                sth = sin(robotCurrentPose(3));
                
                dx = robotDiameter * cth;
                dy = robotDiameter * sth;
                
                xVec = [ robotCurrentLocation(1)   robotCurrentLocation(1)+dx];
                yVec = [ robotCurrentLocation(2)   robotCurrentLocation(2)+dy];
            end
            
            function [x, y] = getCircleCoordinate(pose, r)
                %getCircleCoordinate Compute XData and YData for a circle
                %   This function computes the XData and YData for updating the
                %   graphics object.
                theta = linspace(0,2*pi,20);
                x = r*cos(theta) + pose(1);
                y = r*sin(theta) + pose(2);
            end
            
            function [xVec, yVec] = getVehicleBody(robotCurrentPose, robotDiameter)
                angle = robotCurrentPose(3);
                vehicle_body = [-robotDiameter*2 robotDiameter*2 -robotDiameter*2 ...
                    robotDiameter*2 -robotDiameter*2 -robotDiameter*2 ...
                    robotDiameter*2 robotDiameter*2; ...
                    robotDiameter robotDiameter -robotDiameter -robotDiameter ...
                    robotDiameter -robotDiameter robotDiameter -robotDiameter];
                R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
                vehicle_body = R * vehicle_body;
                xVec = vehicle_body(1,:) + robotCurrentPose(1);
                yVec = vehicle_body(2,:) + robotCurrentPose(2);
                
                %             robotCurrentLocation = robotCurrentPose(1:2);
                %             cth = cos(robotCurrentPose(3));
                %             sth = sin(robotCurrentPose(3));
                %
                %             dx = robotDiameter * cth;
                %             dy = robotDiameter * sth;
                %
                %             xVec = [ robotCurrentLocation(1)-dx   robotCurrentLocation(1)+dx];
                %             yVec = [ robotCurrentLocation(2)-dy   robotCurrentLocation(2)+dy];
            end
            
        end
    end
    
