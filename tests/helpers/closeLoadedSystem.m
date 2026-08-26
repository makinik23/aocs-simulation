function closeLoadedSystem(systemName)
% Description:
%   Closes a loaded Simulink model or harness without saving changes.
%
% Arguments:
%   systemName - Model or harness system name.
%
% Outputs:
%   None.

if bdIsLoaded(systemName)
    close_system(systemName, 0);
end
end
