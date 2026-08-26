function closeHarnessAndModel(owner, harnessName, modelName)
% Description:
%   Closes a Simulink Test harness and optionally its owner model without saving.
%
% Arguments:
%   owner - Harness owner block path.
%   harnessName - Harness name to close.
%   modelName - Optional model name to close after the harness.
%
% Outputs:
%   None.

if nargin >= 2 && strlength(string(harnessName)) > 0
    try
        sltest.harness.close(owner, harnessName);
    catch
    end
end

if nargin >= 3
    closeLoadedSystem(modelName);
end
end
