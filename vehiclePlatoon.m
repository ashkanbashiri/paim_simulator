classdef vehiclePlatoon < handle
      
    properties(SetAccess = public)
        totalFuelConsumption = 0;
        pNumber;
        coPlatoon = -1;
        reachedCollisionZone = 0;
        broadcastedOnce = 0;
        finishedCrossing = 0;
        desiredIdleDistance = 10;
        totalWaiting = -1;
        expectedArrivalTime = 0;
        waitingTime = 0;
        delay = 0;
        distt = 123;
        platoonSize;
        numPlatoonPassengers;
        platoonCapacity;
        hasLeader = 0;
        vehicles ;
        isUpdated=0;
        controllers;
        leader;
        linearVelocity = 15;
        angularVelocity = 1;
        lookaheadDistance =1;
        desiredDistance = 20;
        turnNumber;
        distanceToCZ = Inf;
        letgo=0;
        path;
        state ;
        arrivalLane = 1;
        platoonVelocity = 1;
        arrivalTime=Inf;
        timeStep = 0.1;
        color = 'b';
        turn = 'straight';%left %right
        
        spawnPoints = [20 195;%for Lane 1
                        205 20;%for Lane 2
                        380 205;%for Lane 3
                        195 380;%for Lane 4
                        ];
        stopPoints = [187 195;%for Lane 1
                        205 187;%for Lane 2
                        213 205;%for Lane 3
                        195 213;%for Lane 4
                        ];
                        
    end
    
    methods (Access = private)
        function controller = getController(obj)
            controller =  PurePursuit;
            controller.Waypoints = obj.path;
            controller.DesiredLinearVelocity = obj.linearVelocity;
            controller.MaxAngularVelocity = obj.angularVelocity;
            controller.LookaheadDistance = obj.lookaheadDistance;
        end
        
        function res = isArrived(obj)
            res = 0;
            desiredDistanceToStop = 2.5;
            distanceToStopPoint = getDistanceToStop2(obj);
            if distanceToStopPoint < desiredDistanceToStop 
                  res = 1;%The leader is right behind the stop point, the platoon should stop
            else%does not need to reach collision zone -- This is temp code- change later
                  obj.reachedCollisionZone = 1;
                      %fprintf('Reached Collision Zone');
            end 
        end
    end
    methods
        function delete(obj)
            for i=1:obj.platoonSize
                delete(obj.vehicles(i));
            end
        end
        function waiting = getSumOfWaitings(obj)
            waiting = obj.platoonSize*obj.waitingTime;
        end
        function obj = vehiclePlatoon(capacity,leader,turn,arrivalLane,frameCounter,pNumber)
%             fps = 10;
%             switch turn:
%                 case 'straight':
%                                 framesToDestination = fps*(23/15);
%                 case 'right':
%                                 framesToDestination = fps*(23/15);
%                 case 'left':
%                                 framesToDestination = fps*(23/15);
%             end
%             obj.freeFlowArrivalTime = frameCounter + framesToDestination;
%             obj.startingFrame = frameCounter;
            if(nargin==0)
                obj.turn = 0;
            else
                switch turn
                  case 'right'
                      offset=1;
                  case 'straight'
                      offset=2;
                  case 'left'
                      offset=3;
                end
              
              obj.turnNumber = (arrivalLane-1)*3+offset;
              obj.pNumber=pNumber;
              %fprintf('turnnumber = %d',obj.turnNumber);
            obj.coPlatoon = -1;
            obj.turn = turn;
            obj.platoonCapacity = capacity;
            obj.controllers = cell(1,obj.platoonCapacity);
            obj.vehicles = leader;
            obj.platoonSize = 1;
            obj.leader = obj.vehicles(1);
            obj.state = 'moveandwait';
            obj.color = 'b';
            obj.arrivalLane=arrivalLane;
%             if(leader.CurrentPose(1:2) == obj.spawnPoints(1,:))
%                 obj.arrivalLane = 1;
%             elseif (leader.CurrentPose(1:2) == obj.spawnPoints(2,:))
%                 obj.arrivalLane = 2;
%                 elseif (leader.CurrentPose(1:2) == obj.spawnPoints(3,:))
%                     obj.arrivalLane = 3;
%                     elseif (leader.CurrentPose(1:2) == obj.spawnPoints(4,:))
%                         obj.arrivalLane = 4;
%             end
            obj.path = obj.stopPoints(obj.arrivalLane,:);
            obj.arrivalTime = getArrivalTime(obj,frameCounter);
            updateDistanceToCZ(obj)
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function arrivalTime = getArrivalTime(obj,frameCounter)
            arrivalTime = Inf;
            if(strcmp(obj.state,'moveandwait'))
                dist = getDistanceToStop(obj);
                obj.distt = dist;
                temp = (obj.platoonVelocity*obj.timeStep);
                if(temp ==0)
                    vel = obj.platoonVelocity;
                    tstep = obj.timeStep;
                end
                arrivalTime = dist/temp;
                if(obj.expectedArrivalTime==0 && arrivalTime~=Inf )
                    obj.expectedArrivalTime = arrivalTime+frameCounter;
                end
                %arrivalTime=Inf;
            end
            if(strcmp(obj.state,'stopandwait'))  
                dist = getDistanceToStop(obj);
                arrivalTime = dist/(obj.linearVelocity*obj.timeStep);
                arrivalTime=0;
                if(obj.expectedArrivalTime==0 && arrivalTime~=Inf )
                    obj.expectedArrivalTime = arrivalTime+frameCounter;
                end
            end
            arrivalTime = max(arrivalTime,0);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function dist = getDistanceToStop(obj)
            switch obj.arrivalLane
                case 1
                    %disp('lane1');
                    dist = obj.stopPoints(1,1) - obj.vehicles(1).CurrentPose(1);   
                case 2
                    %disp('lane2');
                     dist = obj.stopPoints(2,2) -obj.vehicles(1).CurrentPose(2);
                case 3
                    %disp('lane3');
                    dist = obj.vehicles(1).CurrentPose(1) - obj.stopPoints(3,1);
                case 4
                    %disp('lane4');
                    dist = obj.vehicles(1).CurrentPose(2) - obj.stopPoints(4,2);
            end
        end
        function dist = getDistanceToStop2(obj)
            switch obj.arrivalLane
                case 1
                    %disp('lane1');
                    dist = obj.path(1) - obj.vehicles(1).CurrentPose(1);   
                case 2
                    %disp('lane2');
                     dist = obj.path(2) -obj.vehicles(1).CurrentPose(2);
                case 3
                    %disp('lane3');
                    dist = obj.vehicles(1).CurrentPose(1) - obj.path(1);
                case 4
                    %disp('lane4');
                    dist = obj.vehicles(1).CurrentPose(2) - obj.path(2);
            end
        end
        function setPath(obj,path,frameCounter)
            obj.path = path;
            obj.totalWaiting = obj.waitingTime;
            obj.waitingTime=-1;
            obj.lookaheadDistance =1.3;
            obj.angularVelocity = 1;
            obj.controllers{1} = getController(obj);
            obj.state = 'crossing';
            obj.color = 'g';
%             dist = getDistanceToStop(obj);
%             if(dist>1)
%                 calculatedDelay = (frameCounter + dist/(obj.platoonVelocity*obj.timeStep)) - obj.expectedArrivalTime;
%             else
%                 calculatedDelay = frameCounter - obj.expectedArrivalTime;
%             end
%             obj.delay = calculatedDelay;
        end
        function updateStopPoint(obj,path,lane)
            if(lane==obj.arrivalLane && ...
                    (strcmp(obj.state,'moveandwait') ...
                || strcmp(obj.state,'stopandwait')))
                obj.path = path;
                obj.controllers{1} = getController(obj);
                obj.isUpdated = 1;
                obj.state = 'moveandwait';
            end
        end
        function pose = head(obj)
            pose = obj.leader.CurrentPose;
            [x ,y] = pol2cart(pose(3),0);
            pose = [-x-pose(1) -y-pose(2)];
        end
        
        function tail = tail(obj)
            pose = obj.vehicles(obj.platoonSize).CurrentPose;
            [x ,y] = pol2cart(pose(3),obj.desiredIdleDistance);
            tail = [-x+pose(1) -y+pose(2)];
        end
            
        function addVehicle(obj,num)
            if num+obj.platoonSize > obj.platoonCapacity 
                error('Number of  Vehicles is greater than the capacity');
            end
            for i=1:num
                tail = obj.vehicles(obj.platoonSize).CurrentPose;
                [x ,y] = pol2cart(tail(3),obj.desiredIdleDistance);
                followerPosition = [-x+tail(1) -y+tail(2) tail(3)];
                newFollower = vehicleModel(followerPosition,obj.arrivalLane,i+1,obj.pNumber);
                
                if(i==1)
                    newFollower.role = 'first_follower';
                else
                    newFollower.role = 'follower';
                end
                newFollower.status = obj.state;
                newController =  PurePursuit;
                newController.Waypoints = obj.vehicles(obj.platoonSize).CurrentPose(1:2);
                newController.DesiredLinearVelocity = obj.linearVelocity;
                newController.MaxAngularVelocity = obj.angularVelocity;
                newController.LookaheadDistance = obj.lookaheadDistance;
                obj.platoonSize = obj.platoonSize+1;
                obj.controllers{1,obj.platoonSize} = newController;
                obj.vehicles(obj.platoonSize) = newFollower;
                
            end
           
        end
        function updateDistanceToCZ(obj)
            obj.distanceToCZ = norm(obj.stopPoints(obj.arrivalLane,:)-obj.leader.CurrentPose(1:2));
        end
        function drive(obj,dt,frameCounter,draw)
            obj.timeStep = dt;
            if(obj.isUpdated==1)
                obj.state='moveandwait';
                obj.color = 'b';
                obj.isUpdated = 0;
                if isArrived(obj)
                        obj.state = 'stopandwait';
                        obj.color = 'r';
                        obj.platoonVelocity = 0;
                
                end
            end
            switch (obj.state)
                case 'crossing'
                    if(sqrt(sum((obj.leader.CurrentPose(1:2) - obj.path(end,:)) .^ 2))>10)
                        
                    for i=1:obj.platoonSize
%                         if i>1
%                             obj.controllers{1,i}.Waypoints = obj.vehicles(i-1).CurrentPose(1:2);
%                         end

                        if(i==1)%drive Leader
                            moveto= obj.vehicles(i).vel*1 +10;
                            if(obj.finishedCrossing==1)
                                moveto= obj.vehicles(i).vel*1 +20;
                            end
                            
                            kpw = obj.vehicles(i).kpw;
                            kiw = obj.vehicles(i).kiw;
                            kdw = obj.vehicles(i).kdw;
%                             fprintf('kpw = %.2f, kiw = %.2f, kdw = %.2f\n',kpw,kiw,kdw);
                            
                            %moveto = obj.vehicles(i).vel;
                            %obj.vehicles(i).crossing=1;
                            %obj.vehicles(i).desiredDistanceToPreceding = 0;
                            %moveto =1;
%                             obj.vehicles(i).kpw=0.5;
%                              obj.vehicles(i).kdw=0.1;
%                              obj.vehicles(i).kpx = 0.15;
%                             obj.vehicles(i).kp = .35;
%                             obj.vehicles(i).kplv = 0.1;
                            ttt = obj.vehicles(i).kpw;
                            %fprintf('kpw = %.2f',ttt);
                            %[v, omega] = step(obj.controllers{1,i}, obj.vehicles(i).CurrentPose);
                            %[px py pv pa] = driveLeader(obj.vehicles(i),v,omega,dt,draw,obj.color);
                            if(sqrt(sum((obj.leader.CurrentPose(1:2) - obj.path(1,:)) .^ 2))<8 || obj.letgo==1)
                                tmp = obj.linearVelocity;
                                obj.linearVelocity = tmp+0;
                                obj.letgo=1;
                                if(obj.turnNumber ==1 || obj.turnNumber ==9 ||obj.turnNumber ==11)
                                    %moveto = 5;
                                    [px,py,pv,pa] = drive2(obj.vehicles(i),dt,draw,obj.color,obj.path(end,1),obj.leader.CurrentPose(2)-moveto,obj.linearVelocity,0,obj.linearVelocity,0,obj.state);
                                elseif(obj.turnNumber ==2 || obj.turnNumber ==4 || obj.turnNumber ==12)
                                    %moveto=5;
                                    [px,py,pv,pa] = drive2(obj.vehicles(i),dt,draw,obj.color,obj.leader.CurrentPose(1)+moveto,obj.path(end,2),obj.linearVelocity,0,obj.linearVelocity,0,obj.state);
                                    elseif(obj.turnNumber ==6 || obj.turnNumber ==8 || obj.turnNumber ==10)
                                        %fprintf('going to [%.2f,%.2f]\n',obj.leader.CurrentPose(1)-moveto,obj.path(end,2));
                                        %moveto = 5;
                                        [px,py,pv,pa] = drive2(obj.vehicles(i),dt,draw,obj.color,obj.leader.CurrentPose(1)-moveto,obj.path(end,2),obj.linearVelocity,0,obj.linearVelocity,0,obj.state);
                                        elseif(obj.turnNumber ==3 || obj.turnNumber ==5 || obj.turnNumber ==7)
                                            %moveto = 5;
                                            [px,py,pv,pa] = drive2(obj.vehicles(i),dt,draw,obj.color,obj.path(end,1),obj.leader.CurrentPose(2)+moveto,obj.linearVelocity,0,obj.linearVelocity,0,obj.state);
                                end
                                obj.linearVelocity = tmp;
                            else
                                [px,py,pv,pa] = drive2(obj.vehicles(i),dt,draw,obj.color,obj.path(1,1),obj.path(1,2),obj.linearVelocity,0,obj.linearVelocity,0,obj.state);
                            end

                            %obj.platoonVelocity = v;
                            lv = pv;
                            la = pa;
                        else%drive followers
                            %obj.vehicles(i).kpw=0.5;
                            [px,py,pv,pa] = drive2(obj.vehicles(i),dt,draw,obj.color,px,py,pv,pa,lv,la,obj.state);
                        end
                    end
                    else
                        for i=1:obj.platoonSize
                            obj.vehicles(i).CurrentPose = [1000 1000 0];
                            drive(obj.vehicles(i),0,0,dt,true,obj.color,0,obj.state);
                            saveData(obj.vehicles(i));
                            obj.totalFuelConsumption = obj.totalFuelConsumption +obj.vehicles(i).fuelConsumed;
                        end        
                        obj.state = 'done';
                        obj.color = 'k';
                        %fprintf('Arrival Lane = %d DONE!\n',obj.arrivalLane);
                    end
                    %drawnow;
                case 'moveandwait'
                    precedingState = 0;
                    if isArrived(obj)
                        obj.state = 'stopandwait';
                       
                        obj.color = 'r';
                        obj.platoonVelocity = 0;
                        %fprintf('Arrival Lane = %d Stopped!\n',obj.arrivalLane);
                    else
                        %disp('Not Arrived Yet!');
                    for i=1:obj.platoonSize
                        if (i==1)
                            obj.vehicles(i).kpw=0.5;
                            obj.vehicles(i).kdw=0.1;
                            obj.vehicles(i).kpx = 0.35;
                            obj.vehicles(i).kp = .35;
                            obj.vehicles(i).kplv = 0.1;
                            %newController = obj.controllers{1,i};
%                             newController =  PurePursuit;
%                             newController.DesiredLinearVelocity = obj.linearVelocity;
%                             newController.MaxAngularVelocity = obj.angularVelocity;
%                             newController.LookaheadDistance = obj.lookaheadDistance;
%                             [x ,y] = pol2cart(obj.vehicles(i).CurrentPose(3),obj.desiredDistance);
%                             distance = [x+obj.vehicles(i).CurrentPose(1) y+obj.vehicles(i).CurrentPose(2)];
%                             newController.Waypoints = distance;
%                             obj.controllers{1,i} = newController;
%                             [v, omega] = step(obj.controllers{1,i}, obj.vehicles(i).CurrentPose);
%                             obj.platoonVelocity = v;
                            [px,py,pv,pa] = drive2(obj.vehicles(i),dt,draw,obj.color,obj.path(1,1),obj.path(1,2),obj.linearVelocity,0,obj.vehicles(i).vel,0,obj.state);
                            %[px py pv pa] = driveLeader(obj.vehicles(i),v,omega,dt,draw,obj.color);
                            obj.platoonVelocity = obj.vehicles(i).vel;
                            lv = pv;
                            la = pa;
                        else
                            [px,py,pv,pa] = drive2(obj.vehicles(i),dt,draw,obj.color,px,py,pv,pa,lv,la,obj.state);
                            %fprintf('velocity = %.2f, acc = %.2f\n',pv,pa);
                        end
                        
                        
                        
                    end
                    end
                    
                case 'stopandwait'
                    obj.color = 'r';
                    obj.platoonVelocity = 0;
                    obj.waitingTime = obj.waitingTime+1;
                    for i=1:obj.platoonSize
                        if(i==1) 
                            %[px py pv pa] = driveLeader(obj.vehicles(i),0,0,dt,draw,obj.color);
                            %obj.vehicles(i).kpw=0.0;
                            %obj.vehicles(i).kdw=0.0;
                            [px,py,pv,pa] = drive2(obj.vehicles(i),dt,draw,obj.color,obj.path(1),obj.path(2),obj.vehicles(i).vel,0,obj.vehicles(i).vel,0,obj.state);
                            obj.platoonVelocity = obj.vehicles(i).vel;
                            lv = pv;
                            la = pa;
                        else
                            [px,py,pv,pa] = drive2(obj.vehicles(i),dt,draw,obj.color,px,py,pv,pa,lv,la,obj.state);

                        end
                    end
                    
                case 'deleted'
                    %TODO: complete this later
                    
                case 'done'
                    %move platoon out of the simulation space.
%                     for i=1:obj.platoonSize
%                         obj.vehicles(i).CurrentPose = [1000 1000 0];
%                         drive(obj.vehicles(i),0,0,dt,true,obj.color);
%                     end        
            end
           obj.arrivalTime = getArrivalTime(obj,frameCounter);
           updateDistanceToCZ(obj);
           mytail = tail(obj);
           if(obj.broadcastedOnce ==1)
               obj.finishedCrossing = 0;
           else
           if(norm(obj.stopPoints(obj.arrivalLane,:)-mytail)<8)
               obj.finishedCrossing =1;
               obj.broadcastedOnce = 1;
           end
           end
        end
    end
end