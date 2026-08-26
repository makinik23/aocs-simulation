function jd = calendarUtcToJulianDate(epochUtc)
%CALENDARUTCTOJULIANDATE Convert a Gregorian UTC vector to Julian date.

year = floor(double(epochUtc(1)));
month = floor(double(epochUtc(2)));
day = floor(double(epochUtc(3)));

if month <= 2
    year = year - 1;
    month = month + 12;
end

a = floor(year / 100.0);
b = 2.0 - a + floor(a / 4.0);
dayFraction = (double(epochUtc(4)) ...
    + (double(epochUtc(5)) + double(epochUtc(6)) / 60.0) / 60.0) / 24.0;

jd = floor(365.25 * (year + 4716.0)) ...
    + floor(30.6001 * (month + 1.0)) ...
    + day + dayFraction + b - 1524.5;
end
