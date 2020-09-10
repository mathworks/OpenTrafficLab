classdef DrivingStrategy < driving.scenario.MotionStrategy...
        & driving.scenario.mixin.PropertiesInitializableInConstructor
%DrivingStrategy Driving Strategy
%   DrivingStrategy is used to control vehicles in an OppenTrafficLab
%   simulation. The driving logic consists of car following behavior,
%   suitably modified to react to open or closed lanes ahead.
%   To implemnent a custom driving strategy, use this class as a parent
%   class and override the relevant methods
%   
%   strategy = DrivingStrategy(vehicle) creates a nominal driving strategy
%   for the actor vehicle
%   
%   strategy = DrivingStrategy(...,Name,Value) creates a nominal driving strategy
%   for the actor vehicle
% 
%   Overview
%   DrivingStrategy controls the vehicles in its move method, which is
%   called every time the advance method of the drivingScenario being
%   simulated is called. The move method moves the vehicle to the requested
%   time. It does so by setting all states and state dependent variables to
%   the immediatly previous time step, computing the control input for that
%   set of states, and integrating the states forward in time by one time
%   step. The move method perform the following steps
%   
%   1. Check if vehicle has entered the network or is currently in the network. If not break anf continue
%   2. Set the vehicle's states nadd state dependent variables to the
%   correct their value at the correct time step (i.e one time step before the target time the vehicle is being moved to)
%   3. Call determineDrivingMode
%   4. Call determineDrivingInput
%   5. Integrate states from to the requested time
%   6. Update state dependent variables based on the new value for the
%   states
%   
%   DrivingStrategy properties:
%   
%   NextNode - list of Nodes the vehicle will traverse in its path
%   UDStates - user defined states
%   Mode - flag indicating driving mode (i.e. 'CarFollowing', 'ApproachingRedLight')
%   Data - data structure for saving time series info.
%   
%       Parameters:
%   
%   DesiredSpeed - 
%   ReactionTime - 
%   SpeedBounds - 
%   AccBounds - 
%   
%       States:
%   
%   Position - 
%   Velocity - 
%   ForwardVector - 
%   
%       State dependent variable:
%   
%   Station - 
%   Speed - 
%   Node - 
%   
%   DrivingStrategy methods:
%   
%       Public Access methods:
%
%   getStationDistance - 
%   getSpeed -    
%   getPositon -  
%   getUDStates - 
%
%       User defined methods: 
%   
%   updateUDStates -  
%   initializeUDStates -  
%
%       Driving Logic methods: 
%   
%   determineDrivingMode -  
%   determineDrivingInputs -  
%   carFollowing - 
%
%   
%   Note: DrivingStrategy inherits from a MATLAB class meant for internal
%   use. It has been tested in MATLAB 2020b, and may not work in future or
%   earlier releases
%
% Copyright 2020 - 2020 The MathWorks, Inc.
%% Properties
    properties
       %Scenario drivingScenario object the 
       Scenario
       %Data structure to store the time series data of vehicle's states.
       %    Data is a structure where DrivingStrategy store the state information. 
       %    Set the StoreData property to 'true' to store data at all
       %    time steps. The default is to only store the current and previous time
       %    step, because this is needed for simulation.
       Data = struct('Time',[],...
                     'Station',[],...
                     'Speed',[],...
                     'Node',Node.empty,...
                     'UDStates',[],...
                     'Position',[],...
                     'Yaw',[]);
                 
       %NextNode a vector of type Node containing the vehicle's planned path
       NextNode = Node.empty
       
       %UDStates a vector of doubles used for user defined states.
       %    UDStates and the methods initializeUDStates and updateUDStates are
       %    placeholders for states users might want to define in child
       %    classes of DrivingStrategy, and their dynamics. 
       %    initializeUDStates is called when a vehicle enters the network
       %    and updateUDstates is called at the end of each time step, when
       %    all other states have been updated.
       UDStates = [nan] 
       
       %Mode string flag indicating driving logic
       %    Mode is a flag of type string that is set at each timestep by the method
       %    determineDrivingMode. The possible values in the nominal
       %    DrivingStrategy class are 'CarFollowing','ApproachingGreenLight',
       %    and 'ApproachingRedLight'.
       Mode = '';
       
       % Parameters for car following
       
       DesiredSpeed = 10; % [m/s]
       ReactionTime = 0.8; % [s]
       SpeedBounds = [0,15];
       AccBounds = [-5,3];
       
       % CarFollowingModel is a flag property of type string indicating
       % which car following model to use. Options: 'IDM','Gipps'
       CarFollowingModel = 'Gipps'; 
       
       % Flags
       
       %StoreData flag to indicate wether or not to store the time series data of the states
       %    Set to true to store time series data in the Data structure
       StoreData = true;
       
       %StaticLaneKeeping flag to indicate wether or not the lateral
       %dynamics are enabled
       %    Set to true to have the vehicle follow the center of the lane
       %    without worrying about control the sterring of the vehicle.
       %    The lateral control of the vehicle has not yet been tested.
       StaticLaneKeeping = true;
    end
    
    properties (Dependent,Access = protected)
        % Vehicle States
        Position(3,1) double {mustBeReal, mustBeFinite}
        Velocity(3,1) double {mustBeReal, mustBeFinite}
        ForwardVector(3,1)double {mustBeReal, mustBeFinite}
        Speed
    end
    
    properties (Access = protected)
        % Input States
        Acceleration(1,1) double {mustBeReal, mustBeFinite} = 0
        AngularAcceleration(1,1) double {mustBeReal, mustBeFinite} = 0
    end
    properties (Access = protected)
        % Position Dependent Variables
        Node = Node.empty;
        Station = [];
        % Environment Dependent Variables
        IsLeader = false;
        Leader = driving.scenario.Vehicle.empty;
        LeaderSpacing = nan;
    end
    
    %% Constructor
    methods
        function obj = DrivingStrategy(egoActor,varargin)
            obj@driving.scenario.MotionStrategy(egoActor);
            obj@driving.scenario.mixin.PropertiesInitializableInConstructor(varargin{:});
            egoActor.MotionStrategy = obj;
            obj.Scenario = egoActor.Scenario;
            obj.Speed = obj.DesiredSpeed;
            egoActor.IsVisible = false;
        end
    end
    %% Set and Get methods
    methods
        % Position
        function set.Position(obj,pos)
             obj.EgoActor.Position = pos;
        end
        function pos = get.Position(obj)
            pos = nan(length(obj),3);
            for idx = 1:length(obj)
                pos(idx,:) = obj(idx).EgoActor.Position;
            end
        end
        % Velocity
        function set.Velocity(obj,vel)
            obj.EgoActor.Velocity = vel;
        end
        function vel = get.Velocity(obj)
            vel = nan(length(obj),3);
            for idx = 1:length(obj)
                vel(idx,:) = obj(idx).EgoActor.Velocity;
            end
        end
        % Forward Vector
        function set.ForwardVector(obj,dir)
            if all(size(dir)==[3,1])
                dir=dir';
            end
            obj.EgoActor.ForwardVector = dir;
        end
        function dir = get.ForwardVector(obj)
            dir = nan(length(obj),3);
            for idx = 1:length(obj)
                dir(idx,:) = obj(idx).EgoActor.ForwardVector;
            end
        end
        % Speed
        function s = get.Speed(obj)
            s = obj.ForwardVector*obj.Velocity';
        end
        function set.Speed(obj,speed)
            obj.EgoActor.Velocity = obj.EgoActor.ForwardVector*speed;
        end
        
        % Node
        function set.Node(obj,node)
            if node == obj.Node
                return
            end
            
            if ~isempty(obj.Node)
                removeVehicle(obj.Node,obj.EgoActor);
                if ~isempty(node)
                    addVehicle(node,obj.EgoActor);
                end
                obj.Node = node;
            else
                if ~isempty(node)
                    addVehicle(node,obj.EgoActor);
                end
                obj.Node = node;
            end
                
            
        end
        
    end
    %% Public Access Methods
    methods (Access = public)
        function s = getStationDistance(obj,time) 
            if nargin<2 %If no time is given assume current sim time
                time = obj(1).Scenario.SimulationTime;
            end
            dt = obj(1).Scenario.SampleTime;
            for idx = 1:length(obj)
                tIdx = round((time-obj(idx).Data.Time(1))/dt)+1;
                if tIdx>0 && tIdx<length(obj(idx).Data.Time)+1
                    s(idx) = obj(idx).Data.Station(tIdx);
                else
                    error('The requested time is not on record')
                end
            end
        end
        function node = getNode(obj,time)
            if nargin<2 %If no time is given assusme current sim time
                time = obj(1).Scenario.SimulationTime;
            end
            dt = obj(1).Scenario.SampleTime;
            for idx = 1:length(obj)
                tIdx = round((time-obj(idx).Data.Time(1))/dt)+1;
                if tIdx>0 && tIdx<length(obj(idx).Data.Time)+1
                    node(idx) = obj(idx).Data.Node(tIdx);
                else
                    error('The requested time is not on record')
                end
            end
        end
        function v = getSpeed(obj,time)
            if nargin<2 %If no time is given assusme current sim time
                time = obj(1).Scenario.SimulationTime;
            end
            dt = obj(1).Scenario.SampleTime;
            for idx = 1:length(obj)
                tIdx = round((time-obj(idx).Data.Time(1))/dt)+1;
                if tIdx>0 && tIdx<length(obj(idx).Data.Time)+1
                    v(idx) = obj(idx).Data.Speed(tIdx);
                else
                    error('The requested time is not on record')
                end
            end
        end
        function pos = getPosition(obj,time)
            if nargin<2 %If no time is given assusme current sim time
                time = obj(1).Scenario.SimulationTime;
            end
            dt = obj(1).Scenario.SampleTime;
            for idx = 1:length(obj)
                tIdx = round((time-obj(idx).Data.Time(1))/dt)+1;
                if tIdx>0 && tIdx<length(obj(idx).Data.Time)+1
                    pos(idx,:) = obj(idx).Data.Position(tIdx,:);
                else
                    error('The requested time is not on record')
                end
            end
        end
        function states = getUDStates(obj,time)
            if nargin<2 %If no time is given assusme current sim time
                time = obj(1).Scenario.SimulationTime;
            end
            dt = obj(1).Scenario.SampleTime;
            for idx = 1:length(obj)
                tIdx = round((time-obj(idx).Data.Time(1))/dt)+1;
                if tIdx>0 && tIdx<length(obj(idx).Data.Time)+1
                    states(idx,:)  = obj(idx).Data.UDStates(tIdx,:);
                else
                    error('The requested time is not on record')
                end
            end
        end
    end
    %% Update State and Environment Dependent Variables

    methods (Access = protected)
        function updateUDStates(obj,t)
            obj.UDStates = obj.UDStates;
        end
        function initializeUDStates(obj,t)
            obj.UDStates = obj.UDStates;
        end
    end
    
    %% Helper methods
    methods (Access = protected)
              
        function [s,direction,offset]=getLaneInformation(obj)
            [s,direction,offset]=obj.Node.getStationDistance(obj.Position(1:2));
        end
        
        function length = getSegmentLength(obj)
            length = obj.Node.getRoadSegmentLength();
        end
        
        function goToNextNode(obj,SimulationTime)
            if ~isempty(obj.NextNode)
                obj.Node = obj.NextNode(1);
                obj.NextNode(1)=[];
            else
                obj.EgoActor.ExitTime = SimulationTime;
                obj.EgoActor.IsVisible = false;
                obj.Node = Node.empty;
            end
        end
        
        function injectVehicle(obj,t,speed)
            % Set the new node
            goToNextNode(obj)
            % Set the position and velocity
            obj.Position = getRoadCenterFromStation(obj.Node,0);
            if nargin>2
            obj.Speed = speed;
            end
            obj.Station = 0;
            initializeUDStates(obj,t);
            addData(obj,t)
            obj.EgoActor.IsVisible=true;   
            
        end
        
        function addData(obj,t)
            if ~isempty(obj.Data.Time)
                if t~=obj.Data.Time(end)+obj.Scenario.SampleTime
                    error('Can only add data to the following time step')
                end
            end
            
            obj.Data.Time(end+1) = t;
            obj.Data.Station(end+1) = obj.Station;
            obj.Data.Node(end+1) = obj.Node;
            obj.Data.Speed(end+1) = obj.Speed;
            obj.Data.Position(end+1,:) = obj.Position;
            obj.Data.Yaw(end+1) = obj.EgoActor.Yaw;
            obj.Data.UDStates(end+1,:) = obj.UDStates;
            
            % Delete first entry if store data flag is false
            if ~obj.StoreData && length(obj.Data.Time)>2
                obj.Data.Time(1) = [];
                obj.Data.Station(1) = [];
                obj.Data.Node(1) = [];
                obj.Data.Speed(1) = [];
                obj.Data.Yaw(1) = [];
                obj.Data.Position(1,:) = [];
                obj.Data.UDStates(1,:) = [];
                
                
            end
        end
        
        function orientEgoActor(obj,direction,offset)
            if nargin == 1
                [s,direction,offset]= obj.getLaneInformation();
            end
            obj.ForwardVector = [direction,0];
            obj.Position = obj.Position + [offset,0];
        end
        
        function [leader,leaderSpacing] = getLeader(obj,tNow)
            % Get other actors in the node and their stations
            actors = getVehiclesInSegment(obj);
            drivers = [actors.MotionStrategy];
            selfIdx = find(drivers == obj);
            stations = [drivers.Station];
            deltaStations = stations - stations(selfIdx);
            deltaStations(deltaStations<0.1)=inf;
            [leaderSpacing,idx]=min(deltaStations);
            if isinf(leaderSpacing)
                leaderSpacing = nan;
                leader = driving.scenario.Vehicle.empty;
                if ~isempty(obj.NextNode)
                    [sLeader,leader] = obj.NextNode(1).getTrailingVehicleStation(tNow);
                    if ~isempty(leader)
                        leaderSpacing = sLeader-leader.Length +(obj.getSegmentLength()-stations(selfIdx));
                    end
                end
            
            else
                leader = actors(idx);
                leaderSpacing = leaderSpacing-leader.Length;
            end
            
        end
        
        function actors = getVehiclesInSegment(obj)
            actors = obj.Node.getActiveVehicles();
        end
        
        function state = getNextNodeState(obj)
            state = 1;
            if ~isempty(obj.NextNode)
                if ~isempty(obj.NextNode(1).TrafficController)
                    state = obj.NextNode(1).getNodeState();
                end
            end
        end
    end
    %% Nominal Driving Logic
    methods
        function mode = determineDrivingMode(obj,tNow)
            % This function decides which mode(s) the driver is following,
            % and the creates and outputs the function handle to that mode
            
            segLength = obj.getSegmentLength();
            leader = obj.Leader;
            leaderSpacing = obj.LeaderSpacing;
            distToEnd = segLength-obj.Station;
            state = obj.getNextNodeState();
            isLeader = isempty(leader);
            
            
            % Determine the mode
            if state
                if isLeader
                    mode = 'ApproachingGreenLight';
                else
                    mode = 'CarFollowing';
                end
            else
                if isLeader
                    mode = 'ApproachingRedLight';
                else
                    leaderIsPastRoadEnd = getNode(leader.MotionStrategy,tNow)~=obj.Node;
                    if leaderIsPastRoadEnd
                        mode = 'ApproachingRedLight';
                    else
                        mode = 'CarFollowing';
                    end
                end
            end
        end
        
        function inputs = determineDrivingInputs(obj,tNow)
            
            segLength = obj.getSegmentLength();
            leader = obj.Leader;
            distToEnd = segLength-obj.Station;
            
            switch obj.Mode
                case 'CarFollowing'
                    delVel = leader.MotionStrategy.Speed-obj.Speed;
                    leaderSpacing = obj.LeaderSpacing;
                case 'ApproachingRedLight'
                    delVel = 0 - obj.Speed;
                    leaderSpacing = distToEnd;
                case 'ApproachingGreenLight'
                    delVel = 0;
                    leaderSpacing = inf;
            end
            acc = carFollowing(obj,leaderSpacing,obj.Speed,delVel);
            
            %Saturate acc
                %With velocity bounds
            accBounds = (obj.SpeedBounds-obj.Speed)/obj.Scenario.SampleTime;
                % With acceleration bounds
            accBounds = [max(accBounds(1),obj.AccBounds(1)), min(accBounds(2),obj.AccBounds(2))];
            
            acc = min(max(accBounds(1),acc),accBounds(2));
            
            inputs = [acc,0];
            
        end
        
        function acc = carFollowing(obj,spacing,speed,speedDiff)
            if strcmp(obj.CarFollowingModel,'IDM')
                acc = drivingBehavior.intelligentDriverModel(spacing,speed,speedDiff);
            end
            
            if strcmp(obj.CarFollowingModel,'Gipps')
                if mod(obj.Scenario.SimulationTime,obj.ReactionTime)<obj.Scenario.SampleTime
                    acc = drivingBehavior.gippsDriverModel(spacing,speed,speedDiff);
                else
                    acc = obj.Acceleration;
                end
                if ~isreal(acc)
                    acc = 0;
                end
            end
            
        end
        
        
    end
    %% Inherited Abstract Methods (move, restart)
    methods
        
        function running = move(obj,SimulationTime)
            
            dt =  obj.Scenario.SampleTime;
            tNow = SimulationTime - dt;
            tNext = SimulationTime;
            car = obj.EgoActor;
            
            %% Check vehicle's entry time, and manage injection
            
            if tNow>=car.ExitTime || tNow<car.EntryTime
                car.IsVisible = false;
                running = true;%false;
                return
            end
            
            % Check if the vehicle's entry time has just been reached. If so, confirm there is
            % space in the road for it to enter. Otherwise, push entry time
            % back
            
            if (tNow - car.EntryTime>0)&&(tNow - car.EntryTime<dt)
                
                [s,leader] = getTrailingVehicleStation(obj.NextNode(1));
                if isempty(leader)
                    injectVehicle(obj,tNow,obj.Speed);
                else
                    delVel = leader.MotionStrategy.Speed-obj.Speed;
                    spacing = s-leader.Length;
                    [~,v_b,v_a]=drivingBehavior.gippsDriverModel(spacing,obj.Speed,delVel);
                    if v_b>1
                        speed = min(v_b,v_a);
                        injectVehicle(obj,tNow,speed);
                    else
                        car.EntryTime = car.EntryTime+dt;
                        running = true;
                        return;
                    end
                end
                %%
                %c = discretize(s-5,[-inf,5,10,inf]);
%                 switch c
%                     case 1 %
%                         car.EntryTime = car.EntryTime+dt;
%                         running = true;
%                         return;
%                     case 2
%                         if ~isempty(leader)
%                             speed = leader.MotionStrategy.getSpeed(tNow);
%                         else
%                             speed=obj.Speed;
%                         end
%                         injectVehicle(obj,tNow);
%                     case 3
%                         injectVehicle(obj,tNow,obj.Speed);
%                         
%                 end
            end
            
            %% Set vehicle's state and variables to tNow
            % State
            obj.Position = getPosition(obj,tNow);
            obj.Speed = getSpeed(obj,tNow);
            % State dependent variables
            obj.Station = getStationDistance(obj,tNow);
            obj.Node = getNode(obj,tNow);
            obj.UDStates = getUDStates(obj,tNow);
            
            % Environment Dependent Variables
            [obj.Leader,obj.LeaderSpacing]=getLeader(obj,tNow);
            if isempty(obj.Leader)
                obj.IsLeader = true;
            end
            %% Determine Driving Mode
            obj.Mode = determineDrivingMode(obj,tNow);
            
            %% Get Inputs
            inputs = determineDrivingInputs(obj,tNow);
            
            obj.Acceleration = inputs(1);
            obj.AngularAcceleration = inputs(2);
            
            %% Integrate

                % Position and Velocity
            obj.Position = obj.Position + dt*car.Velocity;
            obj.Speed = obj.Speed + dt*obj.Acceleration;
                
                % Forward Vector
            %obj.ForwardVector = ... to be implemented 
            
            %% Update state dependent variables
            % Get new station distance, node and lane information
            [station, direction, offset] = getLaneInformation(obj);
            
            if station > getSegmentLength(obj) % Check if the veh entered a new node
                goToNextNode(obj,tNext)
                if isempty(obj.Node)
                    % The vehicle finished its path
                    running = false;
                    return
                else
                    % Get new station andlane information
                    [station, direction, offset] = getLaneInformation(obj);
                end
                
            end
            obj.Station = station;
            updateUDStates(obj,tNow);
            
            if obj.StaticLaneKeeping %Orient veh along lane if lateral control is deactivated
                obj.orientEgoActor(direction,offset);
            end
            
            addData(obj,tNext);
            running = true;
        end
        
        function restart(obj)
            % Needs to be programmed
            
        end
    end
end

