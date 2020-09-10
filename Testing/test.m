% Copyright 2020 - 2020 The MathWorks, Inc.

s = createDrivingScenario();
s.StopTime = 50;
net = Node(s,s.RoadSegments(1),1);
initVel = 10;
plot(s)

tInjection = 0;
while advance(s)
    if tInjection>3
        car = vehicle(s,'EntryTime',s.SimulationTime+1,'Velocity',[10,0,0]);
        ms = DrivingStrategy(car,'NextNode',net(1),'StoreData',true);
        tInjection = 0;
    end
    tInjection =tInjection+s.SampleTime;
end