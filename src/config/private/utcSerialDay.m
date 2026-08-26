function serialDay = utcSerialDay(epochUtc)
%UTCSERIALDAY Convert a validated UTC vector to MATLAB serial day.

serialDay = datenum(epochUtc(:).');
end
