classdef PlantReadinessTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function paths(testCase)
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            addpath(root); setupAocsPaths(root,true);
            requireDtm2020Native();
            testCase.addTeardown(@() closeLoadedSystem('aocs_plant'));
        end
    end
    methods (Test)
        function scopedRunRecordsEvidenceAndConservesTorqueFreeMotion(testCase)
            original = setupAocsSimulation();
            load_system(original.Model.File);
            stopTime = get_param(original.Model.Name, 'StopTime');
            file = string(tempname) + '.json';
            testCase.addTeardown(@() delete(file));
            payload = struct('extends', fullfile(projectRoot(), 'config', ...
                'scenarios', 'no_disturbance_torques.json'), ...
                'simulation', struct('stop_time_s', 2), ...
                'results', struct('file', 'readiness_torque_free.mat'));
            writeAocsJson(file,payload);
            [~, config, directory] = run_aocs_simulation(file);
            testCase.verifyEqual(config.Sim.StopTime_s, 2);
            testCase.verifyEqual(evalin('base','AOCS'), original);
            testCase.verifyEqual(get_param(original.Model.Name,'StopTime'),stopTime);
            testCase.verifyTrue(isfile(fullfile(directory,'simulation.mat')));
            metadata = jsondecode(fileread(fullfile(directory,'metadata.json')));
            testCase.verifyEqual(string(metadata.Status),"completed");
            testCase.verifyNotEmpty(metadata.SourceSHA256);
            diagnostics = validate_aocs_results(fullfile(directory,'simulation.mat'));
            testCase.verifyTrue(diagnostics.TorqueFree);
            testCase.verifyTrue(diagnostics.EnergyConserved);
            testCase.verifyTrue(diagnostics.MomentumConserved);
        end
    end
end
