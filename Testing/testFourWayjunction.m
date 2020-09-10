% Copyright 2020 - 2020 The MathWorks, Inc.
%%
% Set-up: Create Scenario and Network
s = createFourWayJunctionScenario();
net = createFourWayJunctionNetwork(s);

s.StopTime=100;
s.SampleTime = 0.05;

%%
InjectionRate = [700, 700, 700, 700]; % veh/hour
TurnRatio = [20, 60, 30];
cars = createVehiclesForFourWayJunction(s,net,InjectionRate, TurnRatio);
%%
nodesInJunction = net(9:end);
cliques = [1,1,1,2,2,2,3,3,3,4,4,4];
cycle = [0,15,30,45,60];
traffic = trafficControl.TrafficLight(nodesInJunction,'Cliques',cliques,'Cycle',cycle);

%%
% Record video
recordVid = false;
if recordVid
    vid = VideoWriter('FourWay_Junction_with_lights');
    vid.FrameRate=1/s.SampleTime;
    vid.Quality = 100;
    open(vid)
end

plot(s)
sPlot = gca;
xLim = sPlot.XLim + [-5,5];
yLim = sPlot.YLim + [-5,5];
set(sPlot,'XLim',xLim)
set(sPlot,'YLim',yLim)
%%
 while advance(s)
     set(sPlot,'XLim',xLim)
     set(sPlot,'YLim',yLim)
     traffic.plotOpenPaths()
     %drawnow
     %Record video
     if recordVid
         frame = getframe(gcf);
         writeVideo(vid,frame);
     end
     
 end
 
if recordVid
 close(vid)
end

