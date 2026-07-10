function results = run_orbit_and_environment_tests()
% Description:
%   Runs every test in tests/orbit_and_environment and raises an error if any
%   test fails.
%
% Arguments:
%   None.
%
% Outputs:
%   results - matlab.unittest.TestResult array for orbit/environment tests.

testFolder = fileparts(mfilename("fullpath"));
results = runtests(testFolder);
disp(table(results));

if any([results.Failed])
    error("AOCS:Tests:Failed", "Orbit and environment regression tests failed.");
end
end
