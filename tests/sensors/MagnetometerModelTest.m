classdef MagnetometerModelTest < matlab.unittest.TestCase

    properties
        ProjectRoot
        ModelName = "aocs_plant"
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            % Description:
            %   Locates the project root and adds source/test helper paths.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            testCase.ProjectRoot = projectRoot();
            setupAocsPaths(testCase.ProjectRoot, true);
            setupAocsSimulation(fullfile(testCase.ProjectRoot, ...
                "config", "AocsSimulationConfig.json"));
        end
    end

    methods (TestMethodTeardown)
        function closeModel(testCase)
            % Description:
            %   Closes the AOCS plant model after each test without saving.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            closeLoadedSystem(testCase.ModelName);
        end
    end

    methods (Test)
        function magnetometerPublishesMeasurementBus(testCase)
            % Description:
            %   Verifies that the magnetometer subsystem consumes its sensor
            %   config and publishes a MagnetometerMeasurementBus.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            load_system(fullfile(testCase.ProjectRoot, "models", ...
                testCase.ModelName + ".slx"));

            magnetometer = testCase.ModelName + "/Sensors/Magnetometer";
            sampleHold = find_system(magnetometer, "SearchDepth", 1, ...
                "BlockType", "ZeroOrderHold");
            busAssembly = magnetometer + "/Magnetometer Measurement Bus Assembly";

            testCase.verifyEqual(string(get_param(magnetometer + ...
                "/MagnetometerConfig", "Value")), ...
                "AOCS_SensorConfig.Magnetometer");
            testCase.verifyEqual(string(get_param(magnetometer + ...
                "/MagnetometerConfig", "OutDataTypeStr")), ...
                "Bus: MagnetometerConfigBus");
            testCase.verifyEqual(string(get_param(sampleHold{1}, ...
                "SampleTime")), ...
                "AOCS_SensorConfig.Magnetometer.sample_time_s");
            testCase.verifyEqual(string(get_param(busAssembly, ...
                "OutDataTypeStr")), ...
                "Bus: MagnetometerMeasurementBus");
            testCase.verifyBusInputName(busAssembly, 1, "B_B_T");
            testCase.verifyBusInputName(busAssembly, 2, "valid");
        end

        function sensorsPublishMeasurementBusToGnc(testCase)
            % Description:
            %   Verifies that the Sensors subsystem assembles gyro and
            %   magnetometer measurements into SensorMeasurementBus and feeds
            %   the GNC subsystem.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            load_system(fullfile(testCase.ProjectRoot, "models", ...
                testCase.ModelName + ".slx"));

            busAssembly = testCase.ModelName + ...
                "/Sensors/Sensor Measurement Bus Assembly";

            testCase.verifyEqual(string(get_param(busAssembly, "OutDataTypeStr")), ...
                "Bus: SensorMeasurementBus");
            testCase.verifyBusInputName(busAssembly, 1, "Gyro");
            testCase.verifyBusInputName(busAssembly, 2, "Magnetometer");
            testCase.verifyInputSource(testCase.ModelName + "/GNC", 1, ...
                testCase.ModelName + "/Sensors");
        end
    end

    methods
        function verifyBusInputName(testCase, block, inputIndex, expectedName)
            % Description:
            %   Verifies the signal name attached to a bus-creator input.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %   block - Bus Creator block path.
            %   inputIndex - Input port index to inspect.
            %   expectedName - Expected signal name.
            %
            % Outputs:
            %   None.

            ports = get_param(block, "PortHandles");
            line = get_param(ports.Inport(inputIndex), "Line");

            testCase.verifyGreaterThan(line, 0);
            testCase.verifyEqual(string(get_param(line, "Name")), ...
                string(expectedName));
        end

        function verifyInputSource(testCase, block, inputIndex, expectedSource)
            % Description:
            %   Verifies the source block connected to a subsystem input port.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %   block - Destination block path.
            %   inputIndex - Input port index to inspect.
            %   expectedSource - Expected upstream block path.
            %
            % Outputs:
            %   None.

            ports = get_param(block, "PortHandles");
            line = get_param(ports.Inport(inputIndex), "Line");
            sourcePort = get_param(line, "SrcPortHandle");
            source = getfullname(get_param(sourcePort, "ParentHandle"));

            testCase.verifyEqual(string(source), string(expectedSource));
        end
    end
end
