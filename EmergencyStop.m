classdef EmergencyStop < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    events
        eStopActive
    end
    
    properties
    end
    
    methods
        function triggerEvent(obj)
            notify(obj, 'eStopActive')
        end
        
    end
end

