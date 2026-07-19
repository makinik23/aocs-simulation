classdef SentinelPodErfaEciEcefValidationTest < matlab.unittest.TestCase

    properties
        ProjectRoot
        ReferenceFile
        HarnessFile
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            testCase.ProjectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.ReferenceFile = fullfile(testCase.ProjectRoot, ...
                "validation", "sentinel_pod", "data", "sentinel_erfa_reference.mat");
            testCase.HarnessFile = fullfile(testCase.ProjectRoot, ...
                "tests", "harnesses", "SentinelEciEcefTransformHarness.slx");

            addpath(testCase.ProjectRoot);
            addpath(fullfile(testCase.ProjectRoot, "src", "analysis"));
            addpath(fullfile(testCase.ProjectRoot, "tests", "harnesses"));
        end
    end

    methods (Test)
        function simulinkHarnessReconstructsSentinelEcefFromErfaReference(testCase)
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
            cleanup = onCleanup(@() closeSentinelErfaHarness(harnessName));
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
time_s = ref.time_s(:);
inputDataset = Simulink.SimulationData.Dataset();
inputDataset = inputDataset.addElement(sentinelErfaTimeseries("r_I_m", ref.r_I_erfa_m, time_s));
inputDataset = inputDataset.addElement(sentinelErfaTimeseries("delta_ut1_s", ref.delta_ut1_s(:), time_s));
inputDataset = inputDataset.addElement(sentinelErfaTimeseries("delta_at_s", ref.delta_at_s(:), time_s));
inputDataset = inputDataset.addElement(sentinelErfaTimeseries("polar_motion_rad", ref.polar_motion_rad, time_s));
inputDataset = inputDataset.addElement(sentinelErfaTimeseries("d_cip_rad", ref.d_cip_rad, time_s));
inputDataset = inputDataset.addElement(sentinelErfaTimeseries("t_s", time_s, time_s));
end

function ts = sentinelErfaTimeseries(name, values, time_s)
ts = timeseries(values, time_s(:));
ts.Name = char(name);
end

function configureSentinelErfaHarnessEpoch(harnessName, epochUtcIso)
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
monthNames = ["January", "February", "March", "April", "May", "June", ...
    "July", "August", "September", "October", "November", "December"];
monthName = char(monthNames(monthNumber));
end

function [time_s, values] = sentinelErfaHarnessOutputVector(simOut, signalName)
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

function closeSentinelErfaHarness(harnessName)
if bdIsLoaded(harnessName)
    close_system(harnessName, 0);
end
end
