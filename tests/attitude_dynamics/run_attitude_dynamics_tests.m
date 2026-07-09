function results = run_attitude_dynamics_tests()
% Description:
%   Runs every test in tests/attitude_dynamics and raises an error if any test
%   fails or is incomplete. This provides a short command for CI or manual use.
%
% Arguments:
%   None.
%
% Outputs:
%   results - matlab.unittest.TestResult array for the attitude dynamics tests.

testFolder = fileparts(mfilename("fullpath"));
results = runtests(testFolder);
disp(table(results))

if any([results.Failed]) || any([results.Incomplete])
    error("AOCS:Tests:Failed", "Attitude dynamics regression tests failed.");
end
end
