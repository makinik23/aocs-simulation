classdef ProjectInfrastructureTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function paths(testCase)
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            addpath(root);
            setupAocsPaths(root, true);
            testCase.addTeardown(@() closeLoadedSystem('aocs_plant'));
        end
    end
    methods (Test)
        function inheritanceIsRelativeToParentNotCurrentFolder(testCase)
            root = projectRoot();
            folder = string(tempname); mkdir(folder);
            testCase.addTeardown(@() rmdir(folder, 's'));
            child = fullfile(folder, 'child.json');
            base = struct('extends', fullfile(root, 'config', 'AocsSimulationConfig.json'), ...
                'simulation', struct('stop_time_s', 3));
            writeAocsJson(fullfile(folder, 'simulation.json'), base);
            writeAocsJson(child, struct('extends', 'simulation.json'));
            old = pwd;
            testCase.addTeardown(@() cd(old));
            cd(fullfile(root, 'config')); % Deliberate name collision.
            config = loadAocsSimulationConfig(child, root);
            testCase.verifyEqual(config.Sim.StopTime_s, 3);
        end
        function inheritanceRejectsCyclesWithAliases(testCase)
            folder = string(tempname); mkdir(folder);
            testCase.addTeardown(@() rmdir(folder, 's'));
            file = fullfile(folder, 'loop.json');
            writeAocsJson(file, struct('extends', './loop.json'));
            testCase.verifyError(@() loadAocsSimulationConfig(file), 'AOCS:Config:ExtendsCycle');
        end
        function missingParentHasActionableError(testCase)
            folder = string(tempname); mkdir(folder);
            testCase.addTeardown(@() rmdir(folder, 's'));
            file = fullfile(folder, 'child.json');
            writeAocsJson(file, struct('extends', 'missing.json'));
            testCase.verifyError(@() loadAocsSimulationConfig(file), 'AOCS:Config:MissingFile');
        end
        function experimentDoesNotPublishOrEditModel(testCase)
            original = setupAocsSimulation();
            load_system(original.Model.File);
            stopTime = get_param(original.Model.Name, 'StopTime');
            dirty = get_param(original.Model.Name, 'Dirty');
            file = string(tempname) + '.json';
            testCase.addTeardown(@() delete(file));
            writeAocsJson(file, struct('extends', fullfile(projectRoot(), ...
                'config', 'AocsSimulationConfig.json'), 'simulation', struct('stop_time_s', 2)));
            [input, config] = createAocsSimulationInput(file);
            testCase.verifyClass(input, 'Simulink.SimulationInput');
            testCase.verifyEqual(config.Sim.StopTime_s, 2);
            testCase.verifyEqual(evalin('base', 'AOCS'), original);
            testCase.verifyEqual(get_param(original.Model.Name, 'StopTime'), stopTime);
            testCase.verifyEqual(get_param(original.Model.Name, 'Dirty'), dirty);
        end
        function modelFollowsProjectGuidelines(testCase)
            setupAocsSimulation();
            load_system(fullfile(projectRoot(), 'models', 'aocs_plant.slx'));
            report = checkAocsModel('aocs_plant');
            testCase.verifyTrue(report.Passed);
        end
        function architectureSubsystemsRetainPreviewsAndColors(testCase)
            % Description:
            %   Verifies that the four architecture layers retain content
            %   previews and distinct non-white colors in the saved model.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            setupAocsSimulation();
            load_system(fullfile(projectRoot(), 'models', 'aocs_plant.slx'));
            names = ["Flight Dynamics", "Sensors", "Drivers", "GNC"];
            colors = strings(size(names));
            for index = 1:numel(names)
                block = "aocs_plant/" + names(index);
                testCase.verifyEqual(string(get_param(block, ...
                    'ContentPreviewEnabled')), "on");
                colors(index) = string(get_param(block, 'BackgroundColor'));
                testCase.verifyNotEqual(colors(index), "white");
            end
            testCase.verifyEqual(numel(unique(colors)), numel(names));
        end
        function fileHashDetectsContentChanges(testCase)
            file = string(tempname) + '.json';
            testCase.addTeardown(@() delete(file));
            writeAocsJson(file, struct('value', 1));
            before = aocsFileHash(file);
            writeAocsJson(file, struct('value', 2));
            testCase.verifyNotEqual(aocsFileHash(file), before);
            testCase.verifyEqual(strlength(before), 64);
        end
    end
end
