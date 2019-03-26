function [ result ] = isConflicting( turn1,turn2 )
conflictTable = [
    1 1 1 0 0 0 0 0 1 0 1 0;% ** 1: conflict, 0: no conflict **  Lane 1 going Right
    1 1 1 1 1 1 0 0 1 0 1 1;% ** 1: conflict, 0: no conflict **  Lane 1 going Straight
    1 1 1 0 1 1 1 1 1 0 1 1;% ** 1: conflict, 0: no conflict **  Lane 1 going Left
    0 1 0 1 1 1 0 0 0 0 0 1;% ** 1: conflict, 0: no conflict **  Lane 2 going Right
    0 1 1 1 1 1 1 1 1 0 0 1;% ** 1: conflict, 0: no conflict **  Lane 2 going Straight
    0 1 1 1 1 1 0 1 1 1 1 1;% ** 1: conflict, 0: no conflict **  Lane 2 going Left
    0 0 1 0 1 0 1 1 1 0 0 0;% ** 1: conflict, 0: no conflict **  Lane 3 going Right
    0 0 1 0 1 1 1 1 1 1 1 1;% ** 1: conflict, 0: no conflict **  Lane 3 going Straight
    1 1 1 0 1 1 1 1 1 0 1 1;% ** 1: conflict, 0: no conflict **  Lane 3 going Left
    0 0 0 0 0 1 0 1 0 1 1 1;% ** 1: conflict, 0: no conflict **  Lane 4 going Right
    1 1 1 0 0 1 0 1 1 1 1 1;% ** 1: conflict, 0: no conflict **  Lane 4 going Straight
    0 1 1 1 1 1 0 1 1 1 1 1;% ** 1: conflict, 0: no conflict **  Lane 4 going Left   
];
result = conflictTable(turn1,turn2);


end

