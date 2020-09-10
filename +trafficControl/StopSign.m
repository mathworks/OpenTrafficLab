classdef StopSign < trafficControl.TrafficController
    %STOPSIGN Summary of this class goes here
    %   Detailed explanation goes here
    
    % Copyright 2020 - 2020 The MathWorks, Inc.
    
    properties
        VehicleSequence
        FeedingNodes
        TimeTo
    end
    
    methods
        function obj = StopSign(inputArg1,inputArg2)
            %STOPSIGN Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

