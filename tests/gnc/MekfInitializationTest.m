classdef MekfInitializationTest < matlab.unittest.TestCase
    % Description:
    %   Exercises the saved block-only MEKF initialization and state memory.

    methods (TestClassSetup)
        function prepareEnvironment(~)
            % Description:
            %   Installs production paths and configuration for copied blocks.
            % Arguments:
            %   None.
            % Outputs:
            %   None.
            setupAocsPaths([], true);
            setupAocsSimulation;
        end
    end

    methods (Test)
        function waitsForValidTriadAndHoldsFirstSolution(testCase)
            % Description:
            %   Rejects invalid startup, captures without latency and does not
            %   reinitialize after changing or invalid later TRIAD solutions.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            [out, fixture, prior] = simulateInitialization([0;0;1;1;0;1]);
            verifyHeldState(testCase, out, fixture, prior, 3);
            testCase.verifyEqual(out.pulse.Data(:), logical([0;0;1;0;0;0]));
        end

        function neverInitializesFromInvalidTriad(testCase)
            % Description:
            %   Keeps the uninitialized identity placeholder and uncertainty.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            [out, fixture, prior] = simulateInitialization(zeros(6,1));
            verifyHeldState(testCase, out, fixture, prior, []);
            testCase.verifyFalse(any(out.pulse.Data(:)));
        end

        function initializesAtFirstTickAndResetsOnRestart(testCase)
            % Description:
            %   Checks immediate initialization and fresh memory on restart.
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            % Outputs:
            %   None.
            [out, fixture, prior, repeated] = simulateInitialization(ones(6,1));
            verifyHeldState(testCase, out, fixture, prior, 1);
            testCase.verifyEqual(out.pulse.Data(:), logical([1;0;0;0;0;0]));
            for name = ["quaternion", "bias", "covariance", "initialized", "stateTime", "pulse"]
                testCase.verifyEqual(repeated.(name).Data, out.(name).Data);
            end
        end
    end
end

function [out, fixture, prior, repeated] = simulateInitialization(valid)
% Description:
%   Copies the saved estimator into a periodic harness with changing TRIAD.
% Arguments:
%   valid - Six TRIAD validity samples at 0.1-second intervals.
% Outputs:
%   out - Logged state and one-time initialization request.
%   fixture - Time and quaternion samples supplied to TRIAD input.
%   prior - Nonzero bias and anisotropic covariance used to test configuration.
%   repeated - Optional repeated run of the same harness.
load_system(fullfile(projectRoot(), 'models', 'aocs_plant.slx'));
plantCleanup = onCleanup(@() close_system('aocs_plant', 0));
model = 'MekfInitializationHarness';
new_system(model);
cleanup = onCleanup(@() close_system(model, 0));
set_param(model, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '0.1', 'StopTime', '0.5', 'ReturnWorkspaceOutputs', 'on');
e = [model '/Estimator'];
add_block('aocs_plant/GNC/State Estimation (MEKF)', e);
add_block('aocs_plant/GNC/GNC Tick', [model '/Tick']);
add_line(model, 'Tick/1', 'Estimator/Trigger');

fixture.Time = (0:0.1:0.5)';
angles = deg2rad((10:10:60)');
fixture.Quaternion = [cos(angles/2), zeros(6,2), sin(angles/2)];
samples = [fixture.Time, fixture.Quaternion, valid, fixture.Time];
workspace = get_param(model, 'ModelWorkspace');
assignin(workspace, 'triadSamples', samples);
triad = Simulink.Bus.createMATLABStruct('AttitudeInitializationBus');
triad.q_BI = [1;0;0;0];
triad.DCM_BI = eye(3);
assignin(workspace, 'triadBase', triad);
add_block('simulink/Sources/Constant', [model '/TRIAD Base'], ...
    'Value', 'triadBase', 'OutDataTypeStr', 'Bus: AttitudeInitializationBus');
add_block('simulink/Sources/From Workspace', [model '/Samples'], ...
    'VariableName', 'triadSamples', 'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Holding final value');
add_block('simulink/Signal Routing/Demux', [model '/Split Samples'], ...
    'Outputs', '[4 1 1]');
add_block('simulink/Math Operations/Reshape', [model '/Quaternion Column'], ...
    'OutputDimensionality', 'Customize', 'OutputDimensions', '[4 1]');
add_block('simulink/Signal Routing/Bus Assignment', [model '/TRIAD'], ...
    'AssignedSignals', 'q_BI,valid,solution_time_s');
add_line(model, 'Samples/1', 'Split Samples/1');
add_line(model, 'TRIAD Base/1', 'TRIAD/1');
add_line(model, 'Split Samples/1', 'Quaternion Column/1');
add_line(model, 'Quaternion Column/1', 'TRIAD/2');
add_line(model, 'Split Samples/2', 'TRIAD/3');
add_line(model, 'Split Samples/3', 'TRIAD/4');
add_line(model, 'TRIAD/1', 'Estimator/1');
types = {'ReferenceVectorBus', 'SensorReportBus', 'SensorReadStatusBus'};
for k = 1:3
    name = ['Unused Input ' num2str(k)];
    variable = ['unused' num2str(k)];
    assignin(workspace, variable, Simulink.Bus.createMATLABStruct(types{k}));
    add_block('simulink/Sources/Constant', [model '/' name], ...
        'Value', variable, 'OutDataTypeStr', ['Bus: ' types{k}]);
    add_line(model, [name '/1'], ['Estimator/' num2str(k+1)]);
end
add_block('simulink/Sinks/Terminator', [model '/Estimate Monitor']);
add_line(model, 'Estimator/1', 'Estimate Monitor/1');
add_block('simulink/Signal Routing/Bus Selector', [e '/Observe Estimate'], ...
    'OutputSignals', 'q_BI,gyro_bias_B_rad_s,P_error,initialized,valid,update_time_s,omega_BI_B_rad_s');
estimatePort = get_param([e '/AttitudeEstimate'], 'PortHandles');
estimateLine = get_param(estimatePort.Inport, 'Line');
estimateSource = get_param(estimateLine, 'SrcPortHandle');
observerPorts = get_param([e '/Observe Estimate'], 'PortHandles');
add_line(e, estimateSource, observerPorts.Inport);
names = {'quaternion','bias','covariance','initialized','valid','stateTime','rate'};
for k = 1:numel(names)
    add_block('simulink/Sinks/To Workspace', [e '/' names{k}], ...
        'VariableName', names{k}, 'SaveFormat', 'Timeseries');
    add_line(e, ['Observe Estimate/' num2str(k)], [names{k} '/1']);
end
initializer = [e '/Initialization'];
add_block('simulink/Sinks/To Workspace', [initializer '/pulse'], ...
    'VariableName', 'pulse', 'SaveFormat', 'Timeseries');
add_line(initializer, 'Initialize Once/1', 'pulse/1');
config = evalin('base', 'AOCS_GNCConfig');
payload = config.Value;
prior.initial_bias_B_rad_s = [0.001;-0.002;0.003];
prior.initial_covariance = diag([0.01;0.02;0.03;1e-5;2e-5;3e-5]);
payload.MEKF.initial_bias_B_rad_s = prior.initial_bias_B_rad_s;
payload.MEKF.initial_covariance = prior.initial_covariance;
config = Simulink.Parameter(payload);
config.DataType = 'Bus: GNCConfigBus';
input = Simulink.SimulationInput(model);
input = input.setVariable('AOCS_GNCConfig', config);
out = sim(input);
if nargout > 3
    repeated = sim(input);
end
end

function verifyHeldState(testCase, out, fixture, prior, first)
% Description:
%   Checks every state before/after initialization, including no false validity.
% Arguments:
%   testCase - matlab.unittest.TestCase instance.
%   out - Logged estimator result.
%   fixture - Source time and quaternion history.
%   prior - Expected configured bias and covariance.
%   first - First valid sample index, or empty when all samples are invalid.
% Outputs:
%   None.
count = numel(fixture.Time);
expectedQ = repmat([1;0;0;0], 1, count);
expectedTime = zeros(count,1);
expectedInitialized = false(count,1);
if ~isempty(first)
    expectedQ(:,first:end) = repmat(fixture.Quaternion(first,:)', 1, count-first+1);
    expectedTime(first:end) = fixture.Time(first);
    expectedInitialized(first:end) = true;
end
testCase.verifyEqual(out.quaternion.Time(:), fixture.Time, 'AbsTol', 1e-12);
testCase.verifyEqual(reshape(out.quaternion.Data,4,[]), expectedQ, 'AbsTol', 1e-12);
testCase.verifyEqual(reshape(out.bias.Data,3,[]), ...
    repmat(prior.initial_bias_B_rad_s,1,count), 'AbsTol', 1e-12);
testCase.verifyEqual(reshape(out.covariance.Data,6,6,[]), ...
    repmat(prior.initial_covariance,1,1,count), 'AbsTol', 1e-12);
testCase.verifyEqual(out.initialized.Data(:), expectedInitialized);
testCase.verifyEqual(out.stateTime.Data(:), expectedTime, 'AbsTol', 1e-12);
testCase.verifyFalse(any(out.valid.Data(:)));
testCase.verifyEqual(out.rate.Data, zeros(size(out.rate.Data)));
end
