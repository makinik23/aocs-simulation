classdef SensorDriverApiTest < matlab.unittest.TestCase
    %SENSORDRIVERAPITEST Consuming API contracts and scheduled sensor polling.

    properties (TestParameter)
        Sensor = {'Gyro', 'Magnetometer', 'CoarseSunSensors', 'GNSS'}
    end

    methods (TestClassSetup)
        function PrepareEnvironment(~)
            % Description:
            %   Loads configuration and bus definitions for production blocks.
            % Arguments:
            %   None.
            % Outputs:
            %   None.
            setupAocsPaths([], true);
            setupAocsSimulation;
        end
    end

    methods (Test)
        function ReadDistinguishesNoDataNewDataAndStale(testCase, Sensor)
            % Description:
            %   Verifies first receipt, identical payloads and consuming reads.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %   Sensor - Parameterized sensor key.
            % Outputs:
            %   None.
            out = SimulateReads(Sensor, '[0.2 0.1]', '0.05', '0.45', 1, false);
            t = out.newData.Time(:);
            expectedNew = abs(t - 0.1) < 1e-10 | abs(t - 0.3) < 1e-10;
            testCase.verifyEqual(out.newData.Data(:), expectedNew);
            received = t >= 0.1 - 1e-10;
            testCase.verifyEqual(out.hasData.Data(:), received);
            testCase.verifyFalse(any(out.overrun.Data(:)));
            testCase.verifyEqual(out.sequence.Data(:), ...
                uint32(received + (t >= 0.3 - 1e-10)));
            testCase.verifyEqual(out.validity.Data(:), double(received));
            values = reshape(out.payload.Data, 3, []).';
            testCase.verifyEqual(values(received, :), repmat([1 2 3], nnz(received), 1));
            testCase.verifyEqual(values(~received, :), zeros(nnz(~received), 3));
        end

        function InvalidReceiptIsStillNewData(testCase, Sensor)
            % Description:
            %   Keeps acquisition freshness independent of sensor validity.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %   Sensor - Parameterized sensor key.
            % Outputs:
            %   None.
            out = SimulateReads(Sensor, '[0.2 0.1]', '0.05', '0.35', 0, false);
            testCase.verifyEqual(nnz(out.newData.Data), 2);
            testCase.verifyEqual(out.validity.Data(:), zeros(numel(out.validity.Data), 1));
            testCase.verifyTrue(out.hasData.Data(end));
        end

        function OverrunReturnsLatestAndClearsOnRead(testCase, Sensor)
            % Description:
            %   Checks overwrite detection and acknowledgement independently
            %   of freshness; the latest report survives an overrun.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %   Sensor - Parameterized sensor key.
            % Outputs:
            %   None.
            out = SimulateReads(Sensor, '0.1', '[0.15 0.05]', '0.5', 1, false);
            testCase.verifyEqual(out.sequence.Data(:), uint32([1; 3; 4; 6]));
            testCase.verifyEqual(out.receiptTime.Data(:), [0; 0.2; 0.3; 0.5], 'AbsTol', 1e-12);
            testCase.verifyEqual(out.overrun.Data(:), logical([0; 1; 0; 1]));
            testCase.verifyTrue(all(out.newData.Data(:)));
        end

        function SequenceWrapDoesNotLookLikeNoData(testCase, Sensor)
            % Description:
            %   Verifies status through uint32 receipt counter wrap to zero.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %   Sensor - Parameterized sensor key.
            % Outputs:
            %   None.
            out = SimulateReads(Sensor, '0.1', '0.1', '0.2', 1, true);
            testCase.verifyEqual(out.sequence.Data(:), uint32([4294967295; 0; 1]));
            testCase.verifyTrue(all(out.hasData.Data(:)));
            testCase.verifyTrue(all(out.newData.Data(:)));
            testCase.verifyFalse(any(out.overrun.Data(:)));
        end

        function CapturePrecedesCoincidentReadAndSimulationResets(testCase, Sensor)
            % Description:
            %   Checks same-tick ordering and clean mailbox state on restart.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %   Sensor - Parameterized sensor key.
            % Outputs:
            %   None.
            [out, repeated] = SimulateReads(Sensor, '0.1', '0.1', '0.3', 1, false);
            testCase.verifyEqual(out.sequence.Data(:), uint32((1:4)'));
            testCase.verifyEqual(out.receiptTime.Data(:), out.receiptTime.Time(:), 'AbsTol', 1e-12);
            testCase.verifyTrue(all(out.newData.Data(:)));
            testCase.verifyFalse(any(out.overrun.Data(:)));
            testCase.verifyEqual(repeated.sequence.Data, out.sequence.Data);
            testCase.verifyEqual(repeated.newData.Data, out.newData.Data);
        end

        function ProductionSensorsHaveIndependentFreshness(testCase)
            % Description:
            %   Exercises all saved Sensors, Drivers and Sensor Access blocks
            %   with independent periods and a configured 20 Hz GNC tick.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            periods = [0.1 0.2 0.25 1.0];
            out = SimulateChain(0.05, periods);
            VerifyChain(testCase, out, 0.05, periods);
            testCase.verifyFalse(any(out.GNSS_validity.Data(:)));
            testCase.verifyEqual(nnz(out.GNSS_newData.Data), 2);
        end

        function SlowGncReportsOverrunsPerSensor(testCase)
            % Description:
            %   Verifies that fast-sensor overruns do not consume or corrupt
            %   the independent status of slower sensors in the same cycle.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            periods = [0.05 0.1 0.2 1.0];
            out = SimulateChain(0.2, periods);
            VerifyChain(testCase, out, 0.2, periods);
        end

        function ConfigurationRejectsInvalidGncPeriods(testCase)
            % Description:
            %   Rejects nonpositive GNC periods before creating a scheduler.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            file = string(tempname) + '.json';
            cleanup = onCleanup(@() delete(file));
            for period = [0 -0.1]
                override = struct('extends', fullfile(projectRoot(), ...
                    'config', 'AocsSimulationConfig.json'), ...
                    'gnc', struct('sample_time_s', period));
                writeAocsJson(file, override);
                testCase.verifyError(@() loadAocsSimulationConfig(file), 'AOCS:Config:InvalidField');
            end
        end
    end
end

function spec = SensorSpec(key)
% Description:
%   Maps a sensor key to saved driver, method, bus and payload names.
% Arguments:
%   key - Sensor key used in the shared buses.
% Outputs:
%   spec - Names required by isolated production-block harnesses.
keys = {'Gyro', 'Magnetometer', 'CoarseSunSensors', 'GNSS'};
names = {'Gyro', 'Magnetometer', 'Coarse Sun Sensors', 'GNSS'};
methods = {'ReadGyro', 'ReadMagnetometer', 'ReadCoarseSunSensors', 'ReadGnss'};
raw = {'GyroMeasurementBus', 'MagnetometerMeasurementBus', 'CoarseSunSensorMeasurementBus', 'GnssMeasurementBus'};
payload = {'omega_rad_s', 'B_B_T', 'sun_B_unit', 'r_I_m'};
index = find(strcmp(keys, key), 1);
spec = struct('Key', key, 'Name', names{index}, 'Method', methods{index}, ...
    'RawBus', raw{index}, 'Payload', payload{index});
end

function [out, repeated] = SimulateReads(key, receiptPeriod, readPeriod, stopTime, valid, wrap)
% Description:
%   Exercises one saved driver and its saved Function Caller in isolation.
% Arguments:
%   key - Sensor key.
%   receiptPeriod - Period and optional phase of receipt events [s].
%   readPeriod - Period and optional phase of consuming calls [s].
%   stopTime - Simulation end time [s], expressed as text.
%   valid - Raw validity, independent of receipt status.
%   wrap - Initializes the counter immediately before uint32 wrap.
% Outputs:
%   out - Reports and flags sampled on every consuming read.
%   repeated - Optional second run to verify reinitialization.
load_system(fullfile(projectRoot(), 'models', 'aocs_plant.slx'));
plantCleanup = onCleanup(@() close_system('aocs_plant', 0));
model = 'SensorDriverApiHarness';
new_system(model);
cleanup = onCleanup(@() close_system(model, 0));
set_param(model, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '0.05', 'StopTime', stopTime, 'ReturnWorkspaceOutputs', 'on', ...
    'EnableMultiTasking', get_param('aocs_plant', 'EnableMultiTasking'));
spec = SensorSpec(key);
add_block(['aocs_plant/Drivers/' spec.Name ' Driver'], [model '/Driver']);
if wrap
    set_param([model '/Driver/Receive Sample/Last Sequence'], ...
        'InitialCondition', 'uint32(4294967294)');
end
measurement = Simulink.Bus.createMATLABStruct(spec.RawBus);
measurement.valid = valid;
measurement.(spec.Payload) = [1;2;3];
assignin(get_param(model, 'ModelWorkspace'), 'measurement', measurement);
add_block('simulink/Sources/Constant', [model '/Measurement'], ...
    'Value', 'measurement', 'OutDataTypeStr', ['Bus: ' spec.RawBus]);
add_block('simulink/Ports & Subsystems/Function-Call Generator', [model '/Receipt'], ...
    'sample_time', receiptPeriod, 'Priority', '10');
add_line(model, 'Measurement/1', 'Driver/1');
add_line(model, 'Receipt/1', 'Driver/2');
add_block('simulink/Sinks/Terminator', [model '/Monitor']);
add_line(model, 'Driver/1', 'Monitor/1');
access = [model '/Poll'];
add_block('simulink/Ports & Subsystems/Subsystem', access);
Simulink.SubSystem.deleteContents(access);
add_block('simulink/Ports & Subsystems/Trigger', [access '/Tick'], 'TriggerType', 'function-call');
add_block(['aocs_plant/GNC/Sensor Access/' spec.Method], [access '/' spec.Method]);
add_block('aocs_plant/GNC/GNC Tick', [model '/GNC Tick'], 'sample_time', readPeriod);
add_line(model, 'GNC Tick/1', 'Poll/Trigger');
LogRead(access, spec, '');
out = sim(model);
if nargout > 1
    repeated = sim(model);
end
end

function LogRead(access, spec, prefix)
% Description:
%   Logs returned values inside the polling task to observe every read once.
% Arguments:
%   access - Harness subsystem containing the production Function Caller.
%   spec - Sensor names from SensorSpec.
%   prefix - Unique prefix for signal and workspace variable names.
% Outputs:
%   None.
add_block('simulink/Signal Routing/Bus Selector', [access '/' prefix 'Flags'], ...
    'OutputSignals', 'has_data,new_data,overrun');
add_block('simulink/Signal Routing/Bus Selector', [access '/' prefix 'Payload'], ...
    'OutputSignals', ['sequence_id,receive_time_s,valid,' spec.Payload]);
add_line(access, [spec.Method '/2'], [prefix 'Flags/1']);
add_line(access, [spec.Method '/1'], [prefix 'Payload/1']);
names = {'hasData', 'newData', 'overrun', 'sequence', 'receiptTime', 'validity', 'payload'};
for index = 1:numel(names)
    name = [prefix names{index}];
    add_block('simulink/Sinks/To Workspace', [access '/' name], ...
        'VariableName', name, 'SaveFormat', 'Timeseries');
    if index <= 3
        source = [prefix 'Flags/' num2str(index)];
    else
        source = [prefix 'Payload/' num2str(index - 3)];
    end
    add_line(access, source, [name '/1']);
end
end

function out = SimulateChain(gncPeriod, sensorPeriods)
% Description:
%   Simulates the complete saved acquisition chain with real sensor blocks.
%   Replaces plant truth and ephemeris references with deterministic fixtures;
%   retains production scheduling, TRIAD and the MEKF interface.
% Arguments:
%   gncPeriod - Polling period [s], supplied through AOCS_GNCConfig.
%   sensorPeriods - Gyro, magnetometer, CSS and GNSS sample periods [s].
% Outputs:
%   out - Per-sensor reports and flags sampled at GNC ticks.
load_system(fullfile(projectRoot(), 'models', 'aocs_plant.slx'));
plantCleanup = onCleanup(@() close_system('aocs_plant', 0));
model = 'SensorDriverApiChainHarness';
new_system(model);
cleanup = onCleanup(@() close_system(model, 0));
set_param(model, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', '0.025', 'StopTime', '1.2', 'ReturnWorkspaceOutputs', 'on', ...
    'EnableMultiTasking', get_param('aocs_plant', 'EnableMultiTasking'));
add_block('aocs_plant/Sensors', [model '/Sensors']);
add_block('aocs_plant/Drivers', [model '/Drivers']);
add_block('aocs_plant/GNC', [model '/GNC']);
for port = 1:5
    add_line(model, ['Sensors/' num2str(port)], ['Drivers/' num2str(port)]);
end
for entry = {'Drivers', 'GNC'}
    ports = get_param([model '/' entry{1}], 'PortHandles');
    for port = 1:numel(ports.Outport)
        name = [entry{1} ' Monitor ' num2str(port)];
        add_block('simulink/Sinks/Terminator', [model '/' name]);
        add_line(model, [entry{1} '/' num2str(port)], [name '/1']);
    end
end
% Only the ephemeris/reference branch is replaced; the GNC scheduler,
% consumers and driver functions are the saved production blocks.
gnc = [model '/GNC'];
reference = [gnc '/Onboard Reference Vectors'];
delete_line(gnc, 'Select Initialization Config/1', 'Onboard Reference Vectors/1');
ports = get_param(reference, 'PortHandles');
delete_line(get_param(ports.Outport, 'Line'));
delete_block(reference);
referenceState = Simulink.Bus.createMATLABStruct('ReferenceVectorBus');
referenceState.sun_I_unit = [1;0;0];
referenceState.B_I_T = [1e-5;2e-5;3e-5];
referenceState.valid = 1;
referenceState.sgp4_sequence_id = uint32(1);
assignin(get_param(model, 'ModelWorkspace'), 'referenceFixture', referenceState);
add_block('simulink/Sources/Constant', [gnc '/Reference Fixture'], ...
    'Value', 'referenceFixture', 'OutDataTypeStr', 'Bus: ReferenceVectorBus');
add_line(gnc, 'Reference Fixture/1', 'TRIAD Initialization/3');
add_line(gnc, 'Reference Fixture/1', 'State Estimation (MEKF)/2');
triad = [gnc '/TRIAD Initialization'];
add_block('simulink/Signal Routing/Bus Selector', [triad '/Observe Initialization'], ...
    'OutputSignals', 'solution_time_s,magnetometer_sequence_id,css_sequence_id');
add_line(triad, 'Attitude Initialization Bus Assembly/1', 'Observe Initialization/1');
for index = 1:3
    logNames = {'triadTime', 'triadMagSequence', 'triadCssSequence'};
    add_block('simulink/Sinks/To Workspace', [triad '/' logNames{index}], ...
        'VariableName', logNames{index}, 'SaveFormat', 'Timeseries');
    add_line(triad, ['Observe Initialization/' num2str(index)], [logNames{index} '/1']);
end
estimator = [gnc '/State Estimation (MEKF)'];
add_block('simulink/Signal Routing/Bus Selector', [estimator '/Observe Received Status'], ...
    'OutputSignals', 'Gyro.new_data');
add_line(estimator, 'SensorReadStatus/1', 'Observe Received Status/1');
add_block('simulink/Sinks/To Workspace', [estimator '/estimatorNewData'], ...
    'VariableName', 'estimatorNewData', 'SaveFormat', 'Timeseries');
add_line(estimator, 'Observe Received Status/1', 'estimatorNewData/1');
state = Simulink.Bus.createMATLABStruct('PlantStateBus');
state.AttitudeState.q_be = [1;0;0;0];
state.AttitudeState.DCM_be = eye(3);
state.AttitudeState.omega_b = [0.01;0.02;0.03];
state.OrbitState.r_I_m = [7e6;0;0];
state.OrbitState.v_I_m_s = [0;7500;0];
state.Environment.B_B_T = [1e-5;2e-5;3e-5];
state.Environment.sun_B_unit = [1;0;0];
state.Environment.sun_visibility = 1;
state.Environment.solar_flux_shadowed_W_m2 = 1000;
assignin(get_param(model, 'ModelWorkspace'), 'fixtureState', state);
add_block('simulink/Sources/Constant', [model '/Truth'], ...
    'Value', 'fixtureState', 'OutDataTypeStr', 'Bus: PlantStateBus');
add_line(model, 'Truth/1', 'Sensors/1');
sensorConfig = evalin('base', 'AOCS_SensorConfig');
keys = {'Gyro', 'Magnetometer', 'CoarseSunSensors', 'GNSS'};
for index = 1:4
    key = keys{index};
    periodField = 'sample_time_s';
    if index == 1
        periodField = 'gyro_sample_time_s';
    end
    sensorConfig.Value.(key).(periodField) = sensorPeriods(index);
    LogRead([gnc '/Sensor Access'], SensorSpec(key), [key '_']);
end
gncConfig = evalin('base', 'AOCS_GNCConfig');
gncConfig.Value.sample_time_s = gncPeriod;
input = Simulink.SimulationInput(model);
input = input.setVariable('AOCS_SensorConfig', sensorConfig);
input = input.setVariable('AOCS_GNCConfig', gncConfig);
out = sim(input);
end

function VerifyChain(testCase, out, gncPeriod, periods)
% Description:
%   Verifies acquisition counts, independent status and current receipt times.
% Arguments:
%   testCase - matlab.unittest.TestCase instance.
%   out - Logged chain result.
%   gncPeriod - Expected GNC polling period [s].
%   periods - Physical sensor sample periods [s].
% Outputs:
%   None.
keys = {'Gyro', 'Magnetometer', 'CoarseSunSensors', 'GNSS'};
testCase.verifyEqual(out.triadTime.Time(:), (0:gncPeriod:1.2)', 'AbsTol', 1e-12);
testCase.verifyEqual(out.triadTime.Data(:), out.triadTime.Time(:), 'AbsTol', 1e-12);
testCase.verifyEqual(out.triadMagSequence.Data, out.Magnetometer_sequence.Data);
testCase.verifyEqual(out.triadCssSequence.Data, out.CoarseSunSensors_sequence.Data);
testCase.verifyEqual(out.estimatorNewData.Time, out.Gyro_newData.Time);
testCase.verifyEqual(out.estimatorNewData.Data, out.Gyro_newData.Data);
for index = 1:4
    key = keys{index};
    series = out.([key '_sequence']);
    t = series.Time(:);
    count = floor((t + 1e-10) / periods(index)) + 1;
    arrivals = diff([0; count]);
    testCase.verifyEqual(t, (0:gncPeriod:1.2)', 'AbsTol', 1e-12);
    testCase.verifyEqual(series.Data(:), uint32(count), key);
    testCase.verifyEqual(out.([key '_receiptTime']).Data(:), ...
        (count - 1) * periods(index), 'AbsTol', 1e-12);
    testCase.verifyEqual(out.([key '_newData']).Data(:), arrivals > 0, key);
    testCase.verifyEqual(out.([key '_overrun']).Data(:), arrivals > 1, key);
    testCase.verifyTrue(all(out.([key '_hasData']).Data(:)), key);
end
end
