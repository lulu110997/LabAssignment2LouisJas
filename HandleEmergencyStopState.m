function HandleEmergencyStopState(~,~)
   % first input - handle to object that triggered the event
   % second input - event.EventData object containing 
   %             information about the event.  
   mydlg = warndlg('E-stop active! To continue program, remove the hazard then turn off E-stop', 'WARNING');
   waitfor(mydlg);
   disp('i am waiting for estop to be turned off')
end

