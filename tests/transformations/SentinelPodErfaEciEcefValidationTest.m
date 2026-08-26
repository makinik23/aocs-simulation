classdef SentinelPodErfaEciEcefValidationTest < matlab.unittest.TestCase

    properties
        ProjectRoot
        ReferenceFile
        HarnessFile
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            % Description:
            %   Locates the project root, Sentinel ERFA fixture, and transform harness.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            testCase.ProjectRoot = projectRoot();
            testCase.ReferenceFile = fullfile(testCase.ProjectRoot, ...
                "validation", "sentinel_pod", "data", "sentinel_erfa_reference.mat");
            testCase.HarnessFile = fullfile(testCase.ProjectRoot, ...
                "tests", "harnesses", "SentinelEciEcefTransformHarness.slx");

            setupAocsPaths(testCase.ProjectRoot, true);
        end
    end

    methods (Test)
        function simulinkHarnessReconstructsSentinelEcefFromErfaReference(testCase)
            % Description:
            %   Runs the ECI/ECEF transform harness against a Sentinel POD/ERFA
            %   reference and checks reconstructed ECEF position error bounds.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            testCase.assertTrue(isfile(testCase.HarnessFile), ...
                "Sentinel ECI/ECEF transform harness is missing.");
            testCase.assertTrue(isfile(testCase.ReferenceFile), ...
                "ERFA reference fixture is missing. Regenerate it with validation/sentinel_pod/generate_sentinel_erfa_reference.py.");

            ref = load(testCase.ReferenceFile);
            testCase.assertEqual(size(ref.r_I_erfa_m, 2), 3);
            testCase.assertEqual(size(ref.r_ECEF_pod_m, 2), 3);
            testCase.assertEqual(numel(ref.time_s), size(ref.r_I_erfa_m, 1));

            harnessName = "SentinelEciEcefTransformHarness";
            load_system(testCase.HarnessFile);
            cleanup = onCleanup(@() closeLoadedSystem(harnessName));
            configureSentinelErfaHarnessEpoch(harnessName, ref.window_start_utc);

            inputDataset = sentinelErfaHarnessInputDataset(ref);
            simIn = Simulink.SimulationInput(harnessName);
            simIn = simIn.setExternalInput(inputDataset);
            simIn = simIn.setModelParameter( ...
                "StartTime", "0", ...
                "StopTime", num2str(ref.time_s(end)), ...
                "Solver", "ode4", ...
                "FixedStep", "10", ...
                "SaveOutput", "on", ...
                "OutputSaveName", "yout", ...
                "SaveFormat", "Dataset");

            simOut = sim(simIn);
            [time_s, r_ECEF_model_m] = sentinelErfaHarnessOutputVector(simOut, "r_ECEF_m");
            r_ECEF_model_m = interp1(time_s, r_ECEF_model_m, ref.time_s(:), "linear");

            error_m = vecnorm(r_ECEF_model_m - ref.r_ECEF_pod_m, 2, 2);
            fprintf("Sentinel ERFA ECI/ECEF error [m]: min=%.6g, max=%.6g, mean=%.6g, median=%.6g, std=%.6g\n", ...
                min(error_m), max(error_m), mean(error_m), median(error_m), std(error_m));

            testCase.verifyLessThanOrEqual(max(error_m), 5e-2, ...
                sprintf("Maximum Sentinel ERFA ECI/ECEF harness error is %.6g m.", max(error_m)));
            testCase.verifyLessThanOrEqual(mean(error_m), 1e-2, ...
                sprintf("Mean Sentinel ERFA ECI/ECEF harness error is %.6g m.", mean(error_m)));
        end
    end
end

function inputDataset = sentinelErfaHarnessInputDataset(ref)
% Description:
%   Builds the external-input Dataset for the Sentinel ECI/ECEF transform harness.
%
% Arguments:
%   ref - Struct loaded from the Sentinel POD/ERFA reference MAT-file.
%
% Outputs:
%   inputDataset - Simulink external-input Dataset for the transform harness.

time_s = ref.time_s(:);
inputDataset = Simulink.SimulationData.Dataset();
inputDataset = inputDataset.addElement(namedTimeseries("r_I_m", ref.r_I_erfa_m, time_s));
inputDataset = inputDataset.addElement(namedTimeseries("delta_ut1_s", ref.delta_ut1_s(:), time_s));
inputDataset = inputDataset.addElement(namedTimeseries("delta_at_s", ref.delta_at_s(:), time_s));
inputDataset = inputDataset.addElement(namedTimeseries("polar_motion_rad", ref.polar_motion_rad, time_s));
inputDataset = inputDataset.addElement(namedTimeseries("d_cip_rad", ref.d_cip_rad, time_s));
inputDataset = inputDataset.addElement(namedTimeseries("t_s", time_s, time_s));
end

function configureSentinelErfaHarnessEpoch(harnessName, epochUtcIso)
% Description:
%   Applies the Sentinel reference epoch to the harness ECI/ECEF transform block.
%
% Arguments:
%   harnessName - Name of the loaded Sentinel transform harness.
%   epochUtcIso - UTC epoch string in ISO-8601 Zulu format.
%
% Outputs:
%   None.

epoch = datetime(string(epochUtcIso), "InputFormat", "yyyy-MM-dd'T'HH:mm:ss'Z'", "TimeZone", "UTC");
epochVec = datevec(epoch);
block = harnessName + "/Direction Cosine Matrix ECI to ECEF";
set_param(block, ...
    "red", "IAU-2000/2006", ...
    "year", sprintf("%.0f", epochVec(1)), ...
    "month", sentinelErfaMonthNumberToName(epochVec(2)), ...
    "day", sprintf("%.0f", epochVec(3)), ...
    "hour", sprintf("%.0f", epochVec(4)), ...
    "min", sprintf("%.0f", epochVec(5)), ...
    "sec", sprintf("%.15g", epochVec(6)), ...
    "deltaT", "Sec", ...
    "errorflag", "Error", ...
    "extraparamflag", "on");
end

function monthName = sentinelErfaMonthNumberToName(monthNumber)
% Description:
%   Converts a numeric month to the Aerospace Blockset mask month name.
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

function [time_s, values] = sentinelErfaHarnessOutputVector(simOut, signalName)
% Description:
%   Extracts one named vector output from the Sentinel transform harness result.
%
% Arguments:
%   simOut - Simulink.SimulationOutput returned by the harness run.
%   signalName - Expected output signal name.
%
% Outputs:
%   time_s - Output sample times [s].
%   values - N-by-3 output vector samples.

yout = simOut.get("yout");
for k = 1:yout.numElements
    element = yout{k};
    if sentinelErfaSignalElementMatches(element, signalName) || yout.numElements == 1
        ts = element.Values;
        time_s = ts.Time(:);
        values = loggedSignalMatrix(ts.Data, 3, signalName);
        return;
    end
end
error("AOCS:Tests:MissingHarnessOutput", ...
    "Could not find harness output '%s'.", char(signalName));
end

function tf = sentinelErfaSignalElementMatches(element, signalName)
% Description:
%   Checks whether a Dataset element corresponds to a requested harness signal.
%
% Arguments:
%   element - Simulink Dataset element.
%   signalName - Expected signal name.
%
% Outputs:
%   tf - True when the element name or block path matches signalName.

tf = false;
if isprop(element, "Name") && string(element.Name) == string(signalName)
    tf = true;
    return;
end

try
    blockPath = string(element.BlockPath.getBlock(1));
    tf = endsWith(blockPath, "/" + string(signalName));
catch
end
end
