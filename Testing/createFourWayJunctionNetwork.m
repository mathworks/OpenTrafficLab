function [net] = createFourWayJunctionNetwork(scenario)

% Copyright 2020 - 2020 The MathWorks, Inc.

%Create the Network of Node Objects
%% Create The Nodes
%Roads
for i=1:4 %into the intersection
net(i) = Node(scenario,scenario.RoadSegments(i),-1);
end

for i=1:4 %out of the intersection
net(end+1) = Node(scenario,scenario.RoadSegments(i),1);
end

% Manuever at the intersection

%South to West 9
net(end+1) = Node(scenario,scenario.RoadSegments(5),1);
%South to North 10
net(end+1) = Node(scenario,scenario.RoadSegments(6),1);
%South to East 11
net(end+1) = Node(scenario,scenario.RoadSegments(7),1);

%West to North 12
net(end+1) = Node(scenario,scenario.RoadSegments(8),-1);
%West to East 13
net(end+1) = Node(scenario,scenario.RoadSegments(9),1);
%West to South 14
net(end+1) = Node(scenario,scenario.RoadSegments(5),-1);

%North to East 15
net(end+1) = Node(scenario,scenario.RoadSegments(10),1);
%North to South 16
net(end+1) = Node(scenario,scenario.RoadSegments(6),-1);
%North to West 17
net(end+1) = Node(scenario,scenario.RoadSegments(8),1);

%East to South 18
net(end+1) = Node(scenario,scenario.RoadSegments(7),-1);
%East to West 19
net(end+1) = Node(scenario,scenario.RoadSegments(9),-1);
%East to North 20
net(end+1) = Node(scenario,scenario.RoadSegments(10),-1);

%% Connect the Nodes
% South Road
net(1).ConnectsTo = net(9);
net(1).ConnectsTo = net(10);
net(1).ConnectsTo = net(11);
net(14).ConnectsTo = net(5);
net(16).ConnectsTo = net(5);
net(18).ConnectsTo = net(5);

net(1).SharesRoadWith = net(4);

% West Road
net(2).ConnectsTo = net(12);
net(2).ConnectsTo = net(13);
net(2).ConnectsTo = net(14);
net(9).ConnectsTo = net(6);
net(17).ConnectsTo = net(6);
net(19).ConnectsTo = net(6);

net(2).SharesRoadWith = net(6);

% North Road
net(3).ConnectsTo = net(15);
net(3).ConnectsTo = net(16);
net(3).ConnectsTo = net(17);
net(10).ConnectsTo = net(7);
net(12).ConnectsTo = net(7);
net(20).ConnectsTo = net(7);

net(3).SharesRoadWith = net(7);

% East Road
net(4).ConnectsTo = net(18);
net(4).ConnectsTo = net(19);
net(4).ConnectsTo = net(20);
net(11).ConnectsTo = net(8);
net(13).ConnectsTo = net(8);
net(15).ConnectsTo = net(8);

net(4).SharesRoadWith = net(8);


end

