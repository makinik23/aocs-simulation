classdef PlantNumericsTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function paths(testCase)
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            addpath(root); setupAocsPaths(root,true); createAocsBuses();
            load_system(fullfile(root,'models','aocs_plant.slx'));
            testCase.addTeardown(@() closeLoadedSystem('aocs_plant'));
        end
    end
    methods (Test)
        function finiteTruthPasses(testCase)
            state = Simulink.Bus.createMATLABStruct('PlantStateBus');
            output = simulateCheck(state);
            testCase.verifyEmpty(output.ErrorMessage);
        end
        function nonfiniteAttitudeFails(testCase)
            state = Simulink.Bus.createMATLABStruct('PlantStateBus');
            output = simulateCheck(state, 'AttitudeState.omega_b', [NaN;0;0]);
            testCase.verifyTrue(contains(string(output.ErrorMessage),'Finite Plant State'), output.ErrorMessage);
        end
        function nonfiniteEnvironmentFails(testCase)
            state = Simulink.Bus.createMATLABStruct('PlantStateBus');
            output = simulateCheck(state, 'Environment.B_B_T', [0;Inf;0]);
            testCase.verifyTrue(contains(string(output.ErrorMessage),'Finite Plant State'), output.ErrorMessage);
        end
    end
end

function output = simulateCheck(state, field, corruption)
[~,name] = fileparts(tempname);
name = "numerics_" + name;
new_system(name);
cleanup = onCleanup(@() close_system(name,0));
add_block('simulink/Sources/Constant',name+'/Truth','Value','testTruth', ...
    'OutDataTypeStr','Bus: PlantStateBus','Position',[30 30 60 60]);
add_block('aocs_plant/Flight Dynamics/Numerical Checks',name+'/Check', ...
    'Position',[180 20 340 80]);
input = Simulink.SimulationInput(name);
if nargin > 1
    % Inject a signal at runtime; Constant bus parameters themselves must be finite.
    add_block('simulink/Signal Routing/Bus Assignment',name+'/Inject Fault', ...
        'AssignedSignals',field,'Position',[100 25 110 75]);
    add_block('simulink/Sources/Constant',name+'/Corruption','Value','testCorrupt', ...
        'Position',[30 120 60 150]);
    add_line(name,'Truth/1','Inject Fault/1');
    add_line(name,'Corruption/1','Inject Fault/2');
    add_line(name,'Inject Fault/1','Check/1');
    input = input.setVariable('testCorrupt',corruption);
else
    add_line(name,'Truth/1','Check/1');
end
input = input.setVariable('testTruth',state);
input = input.setModelParameter('StopTime','0.1','Solver','FixedStepDiscrete', ...
    'FixedStep','0.1','SignalInfNanChecking','none','CaptureErrors','on');
output = sim(input);
end
