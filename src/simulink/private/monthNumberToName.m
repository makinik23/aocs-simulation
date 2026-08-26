function monthName = monthNumberToName(monthNumber)
% Description:
%   Converts a numeric UTC month to the enum string expected by Aerospace
%   Blockset time/date masks.
%
% Arguments:
%   monthNumber - Integer month number in the range 1..12.
%
% Outputs:
%   monthName - Character vector month name.

monthNames = ["January", "February", "March", "April", "May", "June", ...
    "July", "August", "September", "October", "November", "December"];
monthName = char(monthNames(monthNumber));
end
