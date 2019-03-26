classdef intersection < handle
    
    properties(SetAccess = private)
        granularity;
        lineWidth;
        lanePerRoad;
    end
    
        properties(SetAccess = public)
        title = 'Map';
        spawnPoints = [20 195 0;%for Lane 1
                        205 20 pi/2;%for Lane 2
                        380 205 pi;%for Lane 3
                        195 380 -pi/2;%for Lane 4
                        ];
        stopPoints = [180 195;%for Lane 1
                        205 180;%for Lane 2
                        220 205;%for Lane 3
                        195 215;%for Lane 4
                        ];
        endPoints =  [20 205;
                        195 20;
                        380 195;
                        205 380;];
                    
        lane1To4 = [180 195;%Lane 1 Turn left
    190    195;
    195    200;
    200   205;
    202   210;
    205   220;
    205   230;205   240;205   250;205   260;205   270;205   280;205   290;205   300;
    205   310;205   320;205   330;205   340;205   350;205   360;205   370;205 380];
lane1To3 = [180 195;%Lane 1 Go Straight
    190    195;
    195    195;
    200   195;
    205   195;
    220   195;230   195;240   195;250   195;260   195;270   195;280   195;290   195;300  195;
    310   195;320   195;340   195;350   195;360   195;370   195;375   195; 380 195];
lane1To2 = [180 195;%Lane 1 Turn right
    185    195;
    190   195;
    192    195;
    194   185;194   175;194   165;194   155;
    195   145;195   135;195   125;195   115;195   105;195   95;195   85;195   75;195   65;
    195   55;195   45;195   35;195   30;195   20];
lane2To3 = [205 180;%Lane 2 turn right
    205 185;
    205 190;
    205 192;
    215 195;
    225 195;235 195;245 195;255 195;265 195;275 195;285 195;295 195;
    305 195;315 195;325 195;335 195;345 195;355 195;365 195;375 195;380 195];
lane2To4 = [205 180;%Lane 2 go straight
    205 190;
    205 195;
    205 200;
    205 205;
    205 225;205 235;205 245;205 255;205 265;205 275;205 285;205 295;
    205 305;205 315;205 325;205 335;205 345;205 355;205 365;205 375;205 380];
lane2To1 = [205 190;
    205    190;
    205    195;
    200   200;
    190   205;
    170   205;160   205;150   205;140   205;130   205;120   205;110   205;100   205;
    90   205;80   205;70   205;60   205;50   205;40   205;30   205;25   205;20 205];%Lane 2 turn left
lane3To4 = [220 205;
    210 205;
    207 205;
    205 210;
    205 220;
    205 230;205 240;205 250;205 260;205 270;205 280;205 290;205 300;
    205 310;205 320;205 330;205 340;205 350;205 360;205 370;205 375;205 380];%Lane 3 turn right
lane3To1 = [220 205;
    210 205;
    200 205;
    190 205;
    180 205;
    170 205;160 205;150 205;140 205;130 205;120 205;110 205;100 205;
    90 205;80 205;70 205;60 205;50 205;40 205;30 205;25 205;20 205];%Lane 3 go straight
lane3To2 = [220 205;
    205 205;
    200 200;
    195 195;
    195 190;
    195 170;195 160;195 150;195 140;195 130;195 120;195 110;195 100;195 90;
    195 80;195 70;195 60;195 50;195 40;195 30;195 25;195 20];%Lane 3 turn left
lane4To1 = [195 220;
    195 210;
    190 208;
    180 205;
    175 205;
    170 205;160 205;150 205;140 205;130 205;120 205;110 205;100 205;90 205;
    80 205;70 205;60 205;50 205;40 205;30 205;25 205;20 205];%Lane 4 turn right
lane4To2 = [195 220;
    195 210;
    195 205;
    195 200;
    195 195;
    195 170;195 160;195 150;195 140;195 130;195 120;195 110;195 100;195 90;
    195 80;195 70;195 60;195 50;195 40;195 30;195 25;195 20;]%Lane 4 go straight
lane4To3 = [195 220;
    195 205;
    200 200;
    205 195;
    220 195;
    230 195;240 195;250 195;260 195;270 195;280 195;290 195;300 195;310 195;
    320 195;330 195;340 195;350 195;360 195;370 195;375 195;380 195;]%Lane 4 turn left
trajectories;             
        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
methods(Access=public)
function path = getTrajectory(obj,arrivalLane,turn)
              switch turn
                  case 'right'
                      offset=1;
                  case 'straight'
                      offset=2;
                  case 'left'
                      offset=3;
              end
              index = (arrivalLane-1)*3+offset;
              path = obj.trajectories(:,:,index);
              
end
          
end
      methods(Static)
          function obj = intersection()
              obj.granularity = 400;
              obj.lineWidth = 1;
              obj.lanePerRoad = 1;
              obj.trajectories        = obj.lane1To2;%Lane 1 turn right
              obj.trajectories(:,:,2) = obj.lane1To3;%Lane 1 go straight
              obj.trajectories(:,:,3) = obj.lane1To4;%Lane 1 turn left
              obj.trajectories(:,:,4) = obj.lane2To3;%Lane 2 turn right
              obj.trajectories(:,:,5) = obj.lane2To4;%Lane 2 go straight
              obj.trajectories(:,:,6) = obj.lane2To1;%Lane 2 turn left
              obj.trajectories(:,:,7) = obj.lane3To4;%Lane 3 turn right
              obj.trajectories(:,:,8) = obj.lane3To1;%Lane 3 go straight
              obj.trajectories(:,:,9) = obj.lane3To2;%Lane 3 turn left
              obj.trajectories(:,:,10) = obj.lane4To1;%Lane 4 turn right
              obj.trajectories(:,:,11) = obj.lane4To2;%Lane 4 go straight
              obj.trajectories(:,:,12) = obj.lane4To3;%Lane 4 turn left

          end
          
          function map = getMap(varargin)
            %UNTITLED Summary of this function goes here
            %   Detailed explanation goes here
            numvarargs = length(varargin);
            if numvarargs > 3
                error('gridMapGenerator: TooManyInputs', ...
                'This function takes 1-3 input arguments');
            end
            optargs = {200 1 1};
            optargs(1:numvarargs) = varargin;
            [granularity ttl lanePerRoad] = optargs{:};
            obj.granularity = granularity;
            obj.title = ttl;
            obj.lanePerRoad = lanePerRoad;
            %granularity = 200;
            %lineWidth = 1;
            intersectionGridMap = zeros(granularity, granularity);
            bottomOfLane = 0.5*granularity-11;
            topOfLane = 0.5* granularity+11;
%             intersectionGridMap(bottomOfLane:bottomOfLane+lineWidth,1:bottomOfLane)=1;
%             intersectionGridMap(bottomOfLane:bottomOfLane+lineWidth,topOfLane:end)=1;
%             intersectionGridMap(topOfLane:topOfLane+lineWidth,1:bottomOfLane)=1;
%             intersectionGridMap(topOfLane:topOfLane+lineWidth,topOfLane:end)=1;
%             intersectionGridMap(1:bottomOfLane+lineWidth,bottomOfLane:bottomOfLane+lineWidth)=1;
%             intersectionGridMap(topOfLane:end,bottomOfLane:bottomOfLane+lineWidth)=1;
%             intersectionGridMap(1:bottomOfLane,topOfLane:topOfLane+lineWidth)=1;
%             intersectionGridMap(topOfLane:end,topOfLane:topOfLane+lineWidth)=1;
%             intersectionGridMap(0.5*granularity,1:bottomOfLane)=0.5;
%             intersectionGridMap(0.5*granularity,topOfLane+lineWidth:end)=0.5;
%             intersectionGridMap(1:bottomOfLane,0.5*granularity)=0.5;
%             intersectionGridMap(topOfLane+lineWidth:end,0.5*granularity)=0.5;
%             intersectionGridMap(1:bottomOfLane,1:bottomOfLane)=0.7;
%             intersectionGridMap(1:bottomOfLane,topOfLane+lineWidth:end)=0.7;
%             intersectionGridMap(topOfLane+lineWidth:end,1:bottomOfLane)=0.7;
%             intersectionGridMap(topOfLane+lineWidth:end,topOfLane+lineWidth:end)=0.7;
            intersectionGridMap = intersectionGridMap';
            map = OccupancyGrid(intersectionGridMap);
            map.ttl = obj.title;

            end
      end
end

