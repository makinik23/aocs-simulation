function validateUtcEpoch(epochUtc, displayName)
%VALIDATEUTCEPOCH Validate a UTC vector [year month day hour minute second]'.

integerParts = epochUtc(1:5);
if any(abs(integerParts - round(integerParts)) > 0)
    error("AOCS:Config:InvalidEpoch", ...
        "Config field %s must use integer year/month/day/hour/minute values.", displayName);
end

year = epochUtc(1);
month = epochUtc(2);
day = epochUtc(3);
hour = epochUtc(4);
minute = epochUtc(5);
second = epochUtc(6);

if year < 1
    error("AOCS:Config:InvalidEpoch", "Config field %s has invalid year.", displayName);
end

if month < 1 || month > 12
    error("AOCS:Config:InvalidEpoch", "Config field %s has invalid month.", displayName);
end

maxDay = daysInMonth(year, month);
if day < 1 || day > maxDay
    error("AOCS:Config:InvalidEpoch", ...
        "Config field %s has invalid day for the given month/year.", displayName);
end

if hour < 0 || hour > 23
    error("AOCS:Config:InvalidEpoch", "Config field %s has invalid hour.", displayName);
end

if minute < 0 || minute > 59
    error("AOCS:Config:InvalidEpoch", "Config field %s has invalid minute.", displayName);
end

if second < 0 || second >= 60
    error("AOCS:Config:InvalidEpoch", "Config field %s has invalid second.", displayName);
end
end

function dayCount = daysInMonth(year, month)
monthLengths = [31 28 31 30 31 30 31 31 30 31 30 31];
dayCount = monthLengths(month);
if month == 2 && isLeapYear(year)
    dayCount = 29;
end
end

function result = isLeapYear(year)
result = (mod(year, 4) == 0 && mod(year, 100) ~= 0) || mod(year, 400) == 0;
end
