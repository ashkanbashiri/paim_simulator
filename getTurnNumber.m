function [ turnNumber ] = getTurnNumber( arrivalLane,turn )
turnNumber = (arrivalLane-1)*3;
if(strcmp(turn,'right'))
    turnNumber = turnNumber+1;
elseif(strcmp(turn,'straight'))
    turnNumber = turnNumber+2;
    elseif(strcmp(turn,'left'))
    turnNumber = turnNumber+3;
end
    
end

