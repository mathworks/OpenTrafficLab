function s = createFourWayJunctionScenario()
% Copyright 2020 - 2020 The MathWorks, Inc.
roadLengths = [50,50,50,50];
IntCenter = ([-401.4941 2185.809 0]+[-381.8 2208.1 0])/2;
e1 = [-401.4941 2185.809 0]-IntCenter;
e1 = -e1./norm(e1);
e2 = [-402.2 2206.4 0]-IntCenter;
e2 = e2./norm(e2);

e3 = [0,0,1];
translation = -IntCenter;

s = drivingScenario;

%% Lane marking
marking = [laneMarking('Unmarked')
    laneMarking('Unmarked')
    laneMarking('Solid', 'Width', 0.13)
    laneMarking('Dashed', 'Width', 0.13)
    laneMarking('Solid', 'Width', 0.13)
    laneMarking('Unmarked')
    laneMarking('Unmarked')];
laneSpecification = lanespec([3 3], 'Width', [1.5 0.15 3.65 3.65 0.15 1.5], 'Marking', marking);

%% Turn
roadCenters = [-401.4941 2185.809 0;
    -400.5184 2186.908 0;
    -398.6231 2189.23 0;
    -399.4963 2203.927 0;
    -401.6534 2206.009 0;
    -402.7523 2206.984 0];
roadCenters = roadCenters+translation;

roadCenters1 = dot(roadCenters,repmat(e1,[6,1]),2);
roadCenters2 = dot(roadCenters,repmat(e2,[6,1]),2);
roadCenters3 = dot(roadCenters,repmat(e3,[6,1]),2);


%% Roads 
%South Road
locRoadCenters = [roadCenters1(1),roadCenters2(1),roadCenters3(1);roadCenters1(1)-roadLengths(1),roadCenters2(1),roadCenters3(1)];
road(s,locRoadCenters,'Lanes', laneSpecification);

%East Road
locRoadCenters = [roadCenters1(end),roadCenters2(end),roadCenters3(end);roadCenters1(end),roadCenters2(end)+roadLengths(2),roadCenters3(end)];
road(s,locRoadCenters,'Lanes', laneSpecification);

%North Road
locRoadCenters = [-roadCenters1(1),roadCenters2(1),roadCenters3(1);-roadCenters1(1)+roadLengths(1),roadCenters2(1),roadCenters3(1)];
road(s,locRoadCenters,'Lanes', laneSpecification);

%West Road
locRoadCenters = [roadCenters1(end),-roadCenters2(end),roadCenters3(end);roadCenters1(end),-roadCenters2(end)-roadLengths(2),roadCenters3(end)];
road(s,locRoadCenters,'Lanes', laneSpecification);

%South to West
locRoadCenters = [roadCenters1,roadCenters2,roadCenters3];
road(s,locRoadCenters,'Lanes', laneSpecification);

%South to North
locRoadCenters=[roadCenters1(1),roadCenters2(1),roadCenters3(1);-roadCenters1(1),roadCenters2(1),roadCenters3(1)];
road(s,locRoadCenters,'Lanes', laneSpecification);

%South to East
locRoadCenters = [roadCenters1,-roadCenters2,roadCenters3];
road(s,locRoadCenters,'Lanes', laneSpecification);


%North to West
locRoadCenters = [-roadCenters1,roadCenters2,roadCenters3];
road(s,locRoadCenters,'Lanes', laneSpecification);


%East to West
locRoadCenters=[roadCenters1(end),roadCenters2(end),roadCenters3(end);roadCenters1(end),-roadCenters2(end),roadCenters3(end)];
road(s,locRoadCenters,'Lanes', laneSpecification);


%North to East
locRoadCenters = [-roadCenters1,-roadCenters2,roadCenters3];
road(s,locRoadCenters,'Lanes', laneSpecification);




