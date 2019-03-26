function [] = testCACC(laneNumber,turnNumber)
granularity = 400;
%Get Map, Show Map
intersectionData = intersection();
map = intersectionData.getMap(granularity);


show(map)
laneMarkingColor = [0.8784    0.6667    0.0588];
hold on
plot([0,180],[200 200],'Color',laneMarkingColor,'LineWidth',2);
hold on
plot([220,400],[200 200],'Color',laneMarkingColor,'LineWidth',2);
hold on
plot([200,200],[0 180],'Color',laneMarkingColor,'LineWidth',2);
hold on
plot([200,200],[220 400],'Color',laneMarkingColor,'LineWidth',2);
% hold on
% dim2 = [.3 .3 .3 .3];
% annotation('rectangle',dim2,'FaceColor','blue','FaceAlpha',.3)
xs = [0 190 190 185 0];
ys = [0 0 185 190 190];
lightseagreen = [0.1255 0.6980 0.6667];
hold on
fill(xs,ys,lightseagreen);
centroid = ones(2,5)*95;
polygon = [xs; ys]';
cnt=1;
for theta=3*pi/2:-pi/2:pi/2
    hold on
    R_90 = [cos(theta) -sin(theta);sin(theta) cos(theta)];
    rotated = (polygon-centroid')*R_90+centroid';
    if(cnt==1 || cnt==2)
    xs = rotated(:,1)' + ones(1,5)*210;
    else
        xs = rotated(:,1)';
    end
    if(cnt==2 || cnt==3)
    ys = rotated(:,2)'+ones(1,5)*210;
    else
        ys = rotated(:,2)';
    end
    fill(xs,ys,lightseagreen);
    cnt= cnt+1;
end
%title('Platoon-Based Intersection Management');
xlim([0 granularity]);
ylim([0 granularity]);
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
frameCounter = 1;
simulationTime = 0.1;
%Spawn First Platoon
%laneNumber =4;% randi([1 4]);
pNumber = 1;
platoonLeader = vehicleModel(intersectionData.spawnPoints(laneNumber,:),laneNumber,1,pNumber);
%Spawn The followers
platoonSize = 5;
turnDecision = turns{turnNumber};
platoon = vehiclePlatoon(platoonSize,platoonLeader,turnDecision,laneNumber,frameCounter,pNumber);
platoon.addVehicle(platoonSize-1);
F(frameCounter) = getframe(gcf);

while(frameCounter<500)
    if(frameCounter==300)
         platoon.setPath(intersectionData.getTrajectory(platoon.arrivalLane,platoon.turn),frameCounter);
    end
     F(frameCounter) = getframe(gcf);
    if(frameCounter>300)
        %pause(0.1);
    end
drive(platoon,simulationTime,frameCounter,true);
frameCounter = frameCounter+1;
drawnow
%pause(0.01)
end
uniqueNumber = num2str(100);
uniqueNumber = 111;
video = VideoWriter(strcat('Platooning-Test',uniqueNumber),'MPEG-4');
video.FrameRate = 20;
open(video);
writeVideo(video,F);
close(video);
fprintf('saved to file\n');
               