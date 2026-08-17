function time = modelTime(epoch_utc, t_s, longitude_rad)
% Description:
%   Converts an elapsed simulation time to the calendar quantities required
%   by DTM2020 without relying on datetime, so the function remains suitable
%   for MATLAB Function blocks and code generation.
%
% Arguments:
%   epoch_utc - UTC epoch [year month day hour minute second].
%   t_s - Elapsed SI seconds from epoch.
%   longitude_rad - East-positive geodetic longitude [rad].
%
% Outputs:
%   time - Struct containing UTC year, decimal day of year, decimal year,
%          UTC seconds of day, and local solar time [rad].

%#codegen

epoch_utc = epoch_utc(:);
year = floor(epoch_utc(1));
month = floor(epoch_utc(2));
day = floor(epoch_utc(3));

dayOfYearAtEpoch = calendarDayOfYear(year, month, day);
secondsFromYearStart = (dayOfYearAtEpoch - 1.0) * 86400.0 + ...
    epoch_utc(4) * 3600.0 + epoch_utc(5) * 60.0 + epoch_utc(6) + t_s;

while secondsFromYearStart < 0.0
    year = year - 1.0;
    secondsFromYearStart = secondsFromYearStart + daysInYear(year) * 86400.0;
end

yearDuration_s = daysInYear(year) * 86400.0;
while secondsFromYearStart >= yearDuration_s
    secondsFromYearStart = secondsFromYearStart - yearDuration_s;
    year = year + 1.0;
    yearDuration_s = daysInYear(year) * 86400.0;
end

completedDays = floor(secondsFromYearStart / 86400.0);
secondsOfDay = secondsFromYearStart - completedDays * 86400.0;
dayOfYear = completedDays + 1.0 + secondsOfDay / 86400.0;
decimalYear = year + (dayOfYear - 1.0) / daysInYear(year);
localSolarTime_rad = mod(2.0 * pi * secondsOfDay / 86400.0 + longitude_rad, 2.0 * pi);

time = struct();
time.year = year;
time.day_of_year = dayOfYear;
time.decimal_year = decimalYear;
time.seconds_of_day = secondsOfDay;
time.local_solar_time_rad = localSolarTime_rad;
end

function dayOfYear = calendarDayOfYear(year, month, day)
daysBeforeMonth = [0.0, 31.0, 59.0, 90.0, 120.0, 151.0, ...
    181.0, 212.0, 243.0, 273.0, 304.0, 334.0];
dayOfYear = daysBeforeMonth(month) + day;
if month > 2.0 && isLeapYear(year)
    dayOfYear = dayOfYear + 1.0;
end
end

function count = daysInYear(year)
count = 365.0 + double(isLeapYear(year));
end

function result = isLeapYear(year)
result = mod(year, 4.0) == 0.0 && ...
    (mod(year, 100.0) ~= 0.0 || mod(year, 400.0) == 0.0);
end
