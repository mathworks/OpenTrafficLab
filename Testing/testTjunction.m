% Copyright 2020 - 2020 The MathWorks, Inc.
%%
% Set-up: Create Scenario and Network
s = createTJunctionScenario();
net = createTJunctionNetwork(s);

s.StopTime=100;
s.SampleTime = 0.05;

%%
InjectionRate = [700, 700, 700]; % veh/hour
TurnRatio = [40, 60];
fnc = @(varargin) DrivingStrategy(varargin{:},'CarFollowingModel','Gipps');
cars = createVehiclesForTJunction(s,net,InjectionRate, TurnRatio,fnc);
%%
nodesInJunction = net(7:end);
cliques = [1,1,2,2,3,3];
cycle = [0,15,30,45];
traffic = trafficControl.TrafficLight(nodesInJunction,'Cliques',cliques,'Cycle',cycle);

%%
% Record video
recordVid = false;
if recordVid
    vid = VideoWriter('T_Junction_with_lights');
    vid.FrameRate=1/s.SampleTime;
    vid.Quality = 100;
    open(vid)
end

plot(s)
sPlot = gca;
xLim = sPlot.XLim + [-5,5];
yLim = sPlot.YLim + [-5,5];
%%
 while advance(s)
     set(gca,'XLim',xLim)
     set(gca,'YLim',yLim)
     traffic.plotOpenPaths()
     %Record video
     if recordVid
         frame = getframe(gcf);
         writeVideo(vid,frame);
     end
     
 end
 
if recordVid
 close(vid)
end

