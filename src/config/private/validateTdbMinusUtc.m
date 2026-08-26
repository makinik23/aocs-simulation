function validateTdbMinusUtc(value)
%VALIDATETDBMINUSUTC Guard the UTC to TDB offset against unit mistakes.

if value < 0 || value > 200
    error("AOCS:Config:InvalidTimeOffset", ...
        "Config field epoch.tdb_minus_utc_s must be a plausible seconds offset in [0, 200].");
end
end
