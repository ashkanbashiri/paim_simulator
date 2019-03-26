function [F,callCounter,packets,var,AverageDelayPerVehicle,AverageDelayPerPlatoon,totalVehicles,totalVehiclesCrossed] = AIM_Optimal3(policyName,ver,seed,granularity,platoonMaxSize,spawnRate,duration,simSpeed,handles)
tic;
offset = [-10 0 0;0 -10 0;10 0 0;0 10 0];
offset_offset = [-10 0 0;0 -10 0;10 0 0;0 10 0];
rng(seed);
packetsFromPlatoons = 0;
packetsToPlatoons = 0;
%Get Map, Show Map
intersectionData = intersection();
map = intersectionData.getMap(granularity);
show(map)
%title('Platoon-Based Intersection Management');
xlim([0 granularity]);
ylim([0 granularity]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initialization and Constants
laneTraffic = zeros(1,4);
laneIsFull= zeros(1,4);
global simulationTime;
numVehiclesPassed = 0;
numPlatoonsPassed = 0;
currentAvgDelay = 0;
totalVehicles = [0 0 0 0];
totalVehiclesCrossed = [0 0 0 0];
%LaneTail is initialized to the stop points, this will dynamically change..
%as platoon stop for their turn.other platoons will use this data to stop
%right behind the last platoon in queue.
laneStop = [180 195;%for Lane 1
                        205 180;%for Lane 2
                        220 205;%for Lane 3
                        195 215;%for Lane 4
                        ];



turns ={'straight' 'left' 'right'};
numOfPlatoons = 0;
numOfLanes = 4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
frameCounter = 1;
coplatoons = [];
F(frameCounter) = getframe(gcf);
%legend('Vehicle');
%stoppedByUser=0;
tic;
elapsedTime = toc;
%Spawn First Platoon
laneNumber = randi([1 numOfLanes]);
platoonLeader = vehicleModel(intersectionData.spawnPoints(laneNumber,:));
%Spawn The followers
platoonSize = randi([1 platoonMaxSize]);
turnDecision = turns{randi([1 3])};
platoons(numOfPlatoons+1) = vehiclePlatoon(platoonSize,platoonLeader,turnDecision,laneNumber,frameCounter);
numOfPlatoons = numOfPlatoons+1;
platoons(numOfPlatoons).addVehicle(platoonSize-1);
simulationTime = 5*simSpeed;
spawnPerSec = spawnRate/3600;
lastFrame = 0;
clearTime = 0;
times = 0;
laneCapacity = 17;
%legend('Stopped Platoon','Stopped Platoon','Stopped Platoon','Stopped Platoon');
newBatch = 1;
fps=2;%default=2
index=0;
gone = [];
numOfPlatoonsInSchedule=0;
callCounter = 0;
candidates = [0 0 0 0];
while(frameCounter<duration*fps) 
    set(handles.timeLabel,'String',sprintf('%.2fs',frameCounter/fps));
    set(handles.crossedVehicles,'String',sprintf('%d',sum(totalVehiclesCrossed)));
    if(mod(frameCounter,2)==0)%Spawn 4 platoons every 100 frames
    for k=1:4
        %Spawn New Platoons
        %decide lane
        %laneNumber = randi([1 numOfLanes]);
        %decide turn
        turnRand = rand();
        if(k==1 || k==3)
            %Major Road:50% go straight, 25% turn left, 25% turn Right
            if(turnRand<0.6)
                turnDecision = turns{1};%Go Straight
            elseif(turnRand>=0.6 && turnRand<0.8)
                turnDecision = turns{2};%Turn Left
            else
                turnDecision = turns{3};%Turn Right
            end
                
        else
            %Minor Road: 20% go straight, 40% turn left, 40% turn Right
            if(turnRand<0.20)
                turnDecision = turns{1};%Go Straight
            elseif(turnRand>=0.2 && turnRand<0.6)
                turnDecision = turns{2};%Turn Left
            else
                turnDecision = turns{3};%Turn Right
            end
        end
        %turnDecision = turns{randi([1 3])};
        %turnDecision = turns{3};
        %decide platoon size
        platoonSize = randi([1 platoonMaxSize]);
        %Spawn Leader
        prob = rand();
        alreadySpawned = 0;
        %for platoonSize =platoonMaxSize:-1:1
        %Spawn Leader
        
        threshold = spawnPerSec/platoonSize;
        if(prob<threshold && ((laneTraffic(k)+platoonSize)<laneCapacity) && alreadySpawned==0)
            alreadySpawned=1;
            totalVehicles(k) = totalVehicles(k) + platoonSize;
            platoonLeader = vehicleModel(intersectionData.spawnPoints(k,:));
            %Spawn The followers
            platoons(numOfPlatoons+1) = vehiclePlatoon(platoonSize,platoonLeader,turnDecision,k,frameCounter);
            numOfPlatoons = numOfPlatoons+1;
            platoons(numOfPlatoons).addVehicle(platoonSize-1);
            laneTraffic(k) = laneTraffic(k) +platoonSize;
            if(laneTraffic(k)>laneCapacity)
                laneIsFull(k)=1;
            else 
                laneIsFull(k)=0;
            end
%         elseif(prob<threshold)
%             alreadySpawned=1;
%             totalVehicles(k) = totalVehicles(k) + platoonSize;
%             platoonLeader = vehicleModel(intersectionData.spawnPoints(k,:)+offset(k,:));
%             offset(k,:) = offset(k,:) + offset_offset(k,:);
%             %Spawn The followers
%             platoons(numOfPlatoons+1) = vehiclePlatoon(platoonSize,platoonLeader,turnDecision,k,frameCounter);
%             numOfPlatoons = numOfPlatoons+1;
%             platoons(numOfPlatoons).addVehicle(platoonSize-1);
%             laneTraffic(k) = laneTraffic(k) +platoonSize;
%             laneIsFull(k)=1;
        %end
        end
    end
    end
    %Get Solution%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    arrivals = [];
    lengths = [];
    waitings = [];
    packetsFromPlatoons = packetsFromPlatoons + length(platoons);
    indices = 1:length(platoons);
    indices(gone) = [];
    if(isempty(indices))
        drawnow;
        frameCounter = frameCounter+1;
        lastFrame = frameCounter;
        %F(frameCounter) = getframe(gcf);
        continue;
    else
    for j=indices
           
           if (j==1)
               waitings = [1 platoons(j).waitingTime platoons(j).arrivalTime];
               %arrivals = [1 platoons(j).arrivalTime];
           else
               waitings = [waitings; j platoons(j).waitingTime platoons(j).arrivalTime];
               %lengths = [lengths; j platoons(j).platoonSize];
           end
    end
    
    schedule2 = sortrows(waitings,[3]);
    sortedList = [schedule2(:,1)'];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    counter = 0;
    timePassed = frameCounter-lastFrame;  
    lastCandidates = candidates;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(newBatch==1 || ver==2)%index>numOfPlatoonsInSchedule || index==0)
    candidates = [0 0 0 0];
    candidateCounter = 0;
    
        for pt=sortedList
            if(candidateCounter==4)
                break
            else
            lane = platoons(pt).arrivalLane;
%             if(lastFrame==0)
%                 timePassed=0;
%             else
%             timePassed = frameCounter-lastFrame; 
%             end
            if(candidates(lane)==0  && ~strcmp(platoons(pt).state,'crossing') ...
              && ~strcmp(platoons(pt).state,'done')...
              && (platoons(pt).arrivalTime-frameCounter)<=40)
                candidates(lane)=pt;
                platoons(pt).color = 'y';
                candidateCounter = candidateCounter+1;
            end
            end
        end
        candidates = candidates(candidates~=0);
        if(~isempty(candidates))
            if(length(candidates)~=length(lastCandidates))
                newSchedule = greedySort(candidates);
                callCounter = callCounter+1;
                newBatch = 0;
                numOfPlatoonsInSchedule = length(newSchedule);
                index = 1;
            elseif(candidates~=lastCandidates)
                newSchedule = greedySort(candidates);
                callCounter = callCounter+1;
                newBatch = 0;
                numOfPlatoonsInSchedule = length(newSchedule);
                index = 1;
            else 
                newSchedule = greedySort(candidates);
                newBatch = 0;
                numOfPlatoonsInSchedule = length(newSchedule);
                index = 1;
            end
            
        else
            newSchedule = 0;
            index = 0;
        end
        
    end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    coplatooning = 0;
    simultaneous = 0;
    for j=sortedList
        simultaneous = 0;
        if(index>0 && index <=numOfPlatoonsInSchedule && coplatooning==0)
            whosTurn = newSchedule(index);
%             if(sum(find(coplatoons==whosTurn))~=0)
%                 whosTurn = 0;
%             end
        else
            whosTurn=0;
        end
        %&& ~strcmp(platoons(j).state,'done')
       if(isvalid(platoons(j)) )
           if(strcmp(platoons(j).state,'done'))
               gone = [gone j];
               continue;
           end
           timePassed = frameCounter-lastFrame;          
           if(timePassed>=clearTime && j==whosTurn ...
               && (strcmp(platoons(j).state,'stopandwait')...
               || strcmp(platoons(j).state,'moveandwait')))
                index = index + 1;
                if(index>numOfPlatoonsInSchedule)
                    newBatch=1;
                end
                packetsToPlatoons = packetsToPlatoons + 1;
                packetsFromPlatoons = packetsFromPlatoons + 1;
                offset(platoons(j).arrivalLane,:) = offset(platoons(j).arrivalLane,:) - offset_offset(platoons(j).arrivalLane,:);
                platoons(j).setPath(intersectionData.getTrajectory(platoons(j).arrivalLane,platoons(j).turn),frameCounter);
                arrival = platoons(j).arrivalTime;
                totalVehiclesCrossed(platoons(j).arrivalLane) = totalVehiclesCrossed(platoons(j).arrivalLane) + platoons(j).platoonSize;                %counter = counter + 1;
                clearTime = arrival + (25+(platoons(j).platoonSize*8))/(platoons(j).linearVelocity*simulationTime);
                lastFrame = frameCounter;
                laneTraffic(platoons(j).arrivalLane) = laneTraffic(platoons(j).arrivalLane) -platoons(j).platoonSize;
                if(laneTraffic(platoons(j).arrivalLane)>laneCapacity)
                    laneIsFull(platoons(j).arrivalLane)=1;
                else 
                    laneIsFull(platoons(j).arrivalLane)=0;
                end
                %counter = counter + 1;  
                
                for ii= sortedList(find(sortedList==j,1)+1:end)
                   platoons(ii).updateStopPoint(platoons(j).stopPoints(platoons(j).arrivalLane,:),platoons(j).arrivalLane);
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %coCounter=1;
                for coCounter=1:length(platoons(j).coPlatoon)
               if(platoons(j).coPlatoon(coCounter)~=-1 && sum(find(coplatoons==platoons(j).coPlatoon(coCounter)))==0)
           
               temp = platoons(j).coPlatoon(coCounter); %Identify the platoon that has a nonconflicting path with the current platoon
               platoons(j).coPlatoon(coCounter) = -1;
               coplatooning = 1;
              % fprintf('first platoon=%d, turn=%s, second=%d, turn=%s\n',platoons(j).arrivalLane,platoons(j).turn,...
               %    platoons(temp).arrivalLane,platoons(temp).turn);
               
               coplatoons = [coplatoons temp];
               %index = index + 1;
%                 if(index>numOfPlatoonsInSchedule)
%                     newBatch=1;
%                 end
                packetsToPlatoons = packetsToPlatoons + 1;
                packetsFromPlatoons = packetsFromPlatoons + 1;
                offset(platoons(temp).arrivalLane,:) = offset(platoons(temp).arrivalLane,:) - offset_offset(platoons(temp).arrivalLane,:);

                platoons(temp).setPath(intersectionData.getTrajectory(platoons(temp).arrivalLane,platoons(temp).turn),frameCounter);
                arrival = platoons(temp).arrivalTime;
                totalVehiclesCrossed(platoons(temp).arrivalLane) = totalVehiclesCrossed(platoons(temp).arrivalLane) + platoons(temp).platoonSize;                %counter = counter + 1;
                clearTime = max(clearTime, arrival + (25+(platoons(temp).platoonSize*8))/(platoons(temp).linearVelocity*simulationTime));  
                lastFrame = frameCounter;
                laneTraffic(platoons(temp).arrivalLane) = laneTraffic(platoons(temp).arrivalLane) -platoons(temp).platoonSize;
                if(laneTraffic(platoons(temp).arrivalLane)>laneCapacity)
                    laneIsFull(platoons(temp).arrivalLane)=1;
                else 
                    laneIsFull(platoons(temp).arrivalLane)=0;
                end
                %counter = counter + 1;  
                
                for ii= sortedList(find(sortedList==temp,1)+1:end)
                   platoons(ii).updateStopPoint(platoons(temp).stopPoints(platoons(temp).arrivalLane,:),platoons(temp).arrivalLane);
                end
                drive(platoons(temp),simulationTime,frameCounter,true);
                if(strcmp(platoons(temp).state,'stopandwait')...
                   || strcmp(platoons(temp).state,'moveandwait'))
                    for ii= sortedList(find(sortedList==temp,1)+1:end)
                    platoons(ii).updateStopPoint(platoons(temp).tail(),platoons(temp).arrivalLane);
                    end
                end
                platoons(j).color = 'k';
                platoons(temp).color = 'k';
                simultaneous=1;
               end 
                end
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           end
           drive(platoons(j),simulationTime,frameCounter,true);
           if(strcmp(platoons(j).state,'stopandwait')...
                   || strcmp(platoons(j).state,'moveandwait'))
               for ii= sortedList(find(sortedList==j,1)+1:end)
                   platoons(ii).updateStopPoint(platoons(j).tail(),platoons(j).arrivalLane);
               end
           end
           
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       end
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
    %drawnow;
    if(simultaneous==1)
        pause(5);
    end
    frameCounter = frameCounter+1;
    %platoonSizeSetting = platoonMaxSize
    %SimulationTime = frameCounter/2
    F(frameCounter) = getframe(gcf);
    %tic;
    %pause(0.01);
    %times = [times toc-elapsedTime];
    %elapsedTime = toc;
    %platoons = sort(platoons);
    end
end
aveTime = mean(times);
delays = [];
platoonDelays = [];
for j=1:length(platoons)
    if(isvalid(platoons(j)))
        if(strcmp(platoons(j).state,'done') || strcmp(platoons(j).state,'crossing'))
        %totalVehiclesCrossed(platoons(j).arrivalLane) = totalVehiclesCrossed(platoons(j).arrivalLane) + platoons(j).platoonSize;
        delay =platoons(j).totalWaiting/fps;
            if(delay>=0)
                platoonDelays = [platoonDelays delay];
                for(i=1:platoons(j).platoonSize)
                    delays = [delays delay];
                end
            end
        elseif(platoons(j).waitingTime>=0)
            delay = platoons(j).waitingTime/fps;
                platoonDelays = [platoonDelays delay];
                for(i=1:platoons(j).platoonSize)
                    delays = [delays delay];
                end
        end
        delete(platoons(j));
    end
end
delays = delays;
save('alldelays.mat','delays');
platoonDelays = platoonDelays;
AverageDelayPerVehicle = mean(delays);
AverageDelayPerPlatoon = mean(platoonDelays);
packets = packetsFromPlatoons+packetsToPlatoons;
var = sum(delays.^2)/(length(delays)-1) - (length(delays))*mean(delays)^2/(length(delays)-1);
    function sol = getSchedule(lengths,waitingTimes)
      [sorted sol] = sort(waitingTimes,'descend');
    end

function sol = getSchedule2(lengths,arrivalTimes)
      [sorted sol] = sort(arrivalTimes);
    end

    function [newSchedule] = greedySort(candidates)
        minimumDelay = inf;
        newSchedule = candidates;
        clearTimes = getClearTimes(candidates);
        permutations = perms(candidates);
        for i=1:size(permutations,1)
            maxDelay = 0;
            extraWaitTime = max(clearTime - platoons(permutations(i,1)).arrivalTime ,0);
            extraWait = extraWaitTime>0;
            delayPerVehicle = (platoons(permutations(i,1)).waitingTime*extraWait +...
                extraWaitTime)*...
                   platoons(permutations(i,1)).platoonSize;
                
            totalDelay = clearTimes(find(candidates ==permutations(i,1),1))+clearTime+...
                +max(platoons(permutations(i,1)).arrivalTime-clearTime,0);
            %delayPerVehicle = delayPerVehicle+...
             %   platoons(permutations(i,1)).waitingTime*...
              %  platoons(permutations(i,1)).platoonSize;
            %delayPerVehicle=0;
            %delayPerVehicle = delayPerVehicle +...
             %   platoons(permutations(i,1)).waitingTime*...
              %  platoons(permutations(i,1)).platoonSize;
              if(delayPerVehicle>maxDelay)
                  maxDelay = delayPerVehicle;
              end
            for j=2:size(permutations,2)
                extraWaitTime = max(totalDelay - platoons(permutations(i,j)).arrivalTime ,0);
            extraWait = extraWaitTime>0;
                 if(strcmp(policyName,'pdm'))
                 delayPerVehicle =delayPerVehicle+(extraWait*platoons(permutations(i,j)).waitingTime+...
                     extraWaitTime)*...
                   platoons(permutations(i,j)).platoonSize;
                 else
                     delayPerVehicle =delayPerVehicle+ j*(extraWait*platoons(permutations(i,j)).waitingTime+...
                     extraWaitTime)*...
                   platoons(permutations(i,j)).platoonSize;
                 end
               %platoons(permutations(i,j)).waitingTime+
               %laneTraffic(platoons(permutations(i,j)).arrivalLane);

               %platoons(permutations(i,j)).platoonSize; 
               % + ...
                totalDelay = totalDelay+max(platoons(permutations(i,j)).arrivalTime - totalDelay,0) + ...
                    clearTimes(find(candidates ==permutations(i,j),1));
                 if(delayPerVehicle>maxDelay)
                    maxDelay = delayPerVehicle;
                 end
            end
            if(delayPerVehicle<minimumDelay)
                minimumDelay = delayPerVehicle;
                newSchedule = permutations(i,:);
            end
        end
        
        lane_number = platoons(newSchedule(1)).arrivalLane;
        turn = platoons(newSchedule(1)).turn;
        turnNumber = getTurnNumber(lane_number, turn);
        cops = [];
        for my_index=2:length(newSchedule)
            thisTurnNumber = getTurnNumber(platoons(newSchedule(my_index)).arrivalLane,...
                platoons(newSchedule(my_index)).turn);
            if(~isConflicting(turnNumber,thisTurnNumber))
                coplatoon = newSchedule(my_index);
                if length(cops)==0
                    cops =  [cops coplatoon];
                else
                    colisions = 0;
                    for ind=1:length(cops)
                        previousTurnNumber = getTurnNumber(platoons(cops(ind)).arrivalLane,...
                        platoons(cops(ind)).turn);
                        colisions = colisions + isConflicting(previousTurnNumber,thisTurnNumber);
                    end
                    if(colisions==0)
                        cops =  [cops coplatoon];
                    end
                end
                
               %break;
            end
        end
        %cops
        platoons(newSchedule(1)).coPlatoon =  cops;
    end
    function clearTimes = getClearTimes(candidates)
        for i=1:length(candidates)
            if(candidates(i)==0)
                pSize(i) = 0;
            else
                pSize(i) = platoons(candidates(i)).platoonSize;
            end
        end
        constants = zeros(1,length(candidates)).*25;
        clearTimes = (constants+(pSize.*8))./(simulationTime); 
        
    end
end