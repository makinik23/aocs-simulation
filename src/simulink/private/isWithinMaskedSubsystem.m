function tf = isWithinMaskedSubsystem(block, maskType)
% Description:
%   Checks whether a block is nested inside a masked subsystem of a given
%   type. This avoids setting implementation details inside library-linked
%   masked blocks that already expose their own top-level parameters.
%
% Arguments:
%   block - Block path to test.
%   maskType - Ancestor mask type to look for.
%
% Outputs:
%   tf - True when any parent subsystem has the requested mask type.

tf = false;
parent = get_param(block, "Parent");

while strlength(string(parent)) > 0
    try
        if string(get_param(parent, "MaskType")) == string(maskType)
            tf = true;
            return;
        end
        parent = get_param(parent, "Parent");
    catch
        return;
    end
end
end
