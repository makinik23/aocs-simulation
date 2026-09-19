function integrateGnssSensorSimulink(modelFile)
% Description:
%   Adds the sampled GNSS receiver model to Sensors and publishes its
%   measurements through SensorMeasurementBus.
%
% Arguments:
%   modelFile - Optional path to the AOCS plant model.
%
% Outputs:
%   None.

projectRoot = fileparts(fileparts(mfilename("fullpath")));
if nargin < 1
    modelFile = fullfile(projectRoot, "models", "aocs_plant.slx");
end

addpath(projectRoot);
setupAocsPaths(projectRoot);
setupAocsSimulation(fullfile(projectRoot, "config", ...
    "AocsSimulationConfig.json"));

[~, modelName] = fileparts(modelFile);
load_system(modelFile);
cleanup = onCleanup(@() closeIfLoaded(modelName));

replaceGnssSubsystem(modelName);
connectOrbitStateToGnss(modelName);
publishSensorMeasurements(modelName);

set_param(modelName, "SimulationCommand", "update");
save_system(modelName, modelFile);
end

function replaceGnssSubsystem(modelName)
% Description:
%   Replaces any existing GNSS subsystem with the configured measurement model.

sensors = modelName + "/Sensors";
gnss = sensors + "/GNSS";

deleteBlockIfExists(gnss);
add_block("simulink/Ports & Subsystems/Subsystem", gnss, ...
    "Position", [130 567 355 663]);

buildGnssSubsystem(gnss);
end

function buildGnssSubsystem(parent)
% Description:
%   Builds the sampled RTN PVT error model and nominal/dropout/failure logic.

clearSubsystemTemplate(parent);

add_block("simulink/Ports & Subsystems/In1", parent + "/OrbitState", ...
    "Port", "1", "Position", [20 178 50 192]);
add_block("simulink/Signal Routing/Bus Selector", ...
    parent + "/Select Orbit State", ...
    "OutputSignals", "r_I_m,v_I_m_s", ...
    "Position", [95 130 100 240]);

add_block("simulink/Discrete/Zero-Order Hold", ...
    parent + "/Position Sample Hold", ...
    "SampleTime", "AOCS_SensorConfig.GNSS.sample_time_s", ...
    "Position", [145 125 195 155]);
add_block("simulink/Discrete/Zero-Order Hold", ...
    parent + "/Velocity Sample Hold", ...
    "SampleTime", "AOCS_SensorConfig.GNSS.sample_time_s", ...
    "Position", [145 215 195 245]);

errorModel = parent + "/PVT Error Model";
add_block("simulink/Ports & Subsystems/Subsystem", errorModel, ...
    "Position", [255 105 440 265]);
buildPvtErrorModel(errorModel);

add_block("simulink/Sources/Constant", parent + "/GnssConfig", ...
    "Value", "AOCS_SensorConfig.GNSS", ...
    "OutDataTypeStr", "Bus: GnssConfigBus", ...
    "Position", [145 370 195 400]);

fixAvailability = parent + "/Fix Availability";
add_block("simulink/Ports & Subsystems/Subsystem", fixAvailability, ...
    "Position", [255 345 405 425]);
buildFixAvailability(fixAvailability);

add_block("simulink/Discontinuities/Quantizer", ...
    parent + "/Position Quantizer", ...
    "QuantizationInterval", ...
    "AOCS_SensorConfig.GNSS.position_resolution_m", ...
    "Position", [485 125 540 155]);
add_block("simulink/Discontinuities/Quantizer", ...
    parent + "/Velocity Quantizer", ...
    "QuantizationInterval", ...
    "AOCS_SensorConfig.GNSS.velocity_resolution_m_s", ...
    "Position", [485 215 540 245]);

outputSelection = parent + "/Output Selection";
add_block("simulink/Ports & Subsystems/Subsystem", outputSelection, ...
    "Position", [620 105 805 365]);
buildOutputSelection(outputSelection);

add_block("simulink/Signal Routing/Bus Creator", ...
    parent + "/GNSS Measurement Bus Assembly", ...
    "Inputs", "3", ...
    "OutDataTypeStr", "Bus: GnssMeasurementBus", ...
    "NonVirtualBus", "off", ...
    "Position", [880 120 885 350]);
add_block("simulink/Ports & Subsystems/Out1", parent + "/GNSS", ...
    "Port", "1", "Position", [955 228 985 242]);

addNamedLine(parent, "OrbitState/1", "Select Orbit State/1", "OrbitState");
addNamedLine(parent, "Select Orbit State/1", ...
    "Position Sample Hold/1", "r_I_m_truth");
addNamedLine(parent, "Select Orbit State/2", ...
    "Velocity Sample Hold/1", "v_I_m_s_truth");
addNamedLine(parent, "Position Sample Hold/1", "PVT Error Model/1", "");
addNamedLine(parent, "Velocity Sample Hold/1", "PVT Error Model/2", "");
addNamedLine(parent, "PVT Error Model/1", "Position Quantizer/1", "");
addNamedLine(parent, "PVT Error Model/2", "Velocity Quantizer/1", "");
addNamedLine(parent, "Position Quantizer/1", ...
    "Output Selection/1", "r_I_m_nominal");
addNamedLine(parent, "Velocity Quantizer/1", ...
    "Output Selection/2", "v_I_m_s_nominal");
addNamedLine(parent, "GnssConfig/1", "Fix Availability/1", "GNSSConfig");
addNamedLine(parent, "GnssConfig/1", "Output Selection/3", "");
addNamedLine(parent, "Fix Availability/1", "Output Selection/4", ...
    "fix_available");
addNamedLine(parent, "Output Selection/1", ...
    "GNSS Measurement Bus Assembly/1", "r_I_m");
addNamedLine(parent, "Output Selection/2", ...
    "GNSS Measurement Bus Assembly/2", "v_I_m_s");
addNamedLine(parent, "Output Selection/3", ...
    "GNSS Measurement Bus Assembly/3", "valid");
addNamedLine(parent, "GNSS Measurement Bus Assembly/1", ...
    "GNSS/1", "GNSS");
end

function buildPvtErrorModel(parent)
% Description:
%   Builds ionospheric, once-per-orbit, colored, and white RTN errors and
%   maps them onto the sampled inertial truth state.

clearSubsystemTemplate(parent);

add_block("simulink/Ports & Subsystems/In1", parent + "/rTruth_I_m", ...
    "Port", "1", "Position", [20 123 50 137]);
add_block("simulink/Ports & Subsystems/In1", parent + "/vTruth_I_m_s", ...
    "Port", "2", "Position", [20 223 50 237]);

positionError = parent + "/Position Error RTN";
add_block("simulink/Ports & Subsystems/Subsystem", positionError, ...
    "Position", [100 35 300 155]);
buildPositionRtnError(positionError);

velocityError = parent + "/Velocity Error RTN";
add_block("simulink/Ports & Subsystems/Subsystem", velocityError, ...
    "Position", [100 260 300 380]);
buildVelocityRtnError(velocityError);

applyErrors = parent + "/Apply RTN Errors";
add_block("simulink/User-Defined Functions/MATLAB Function", applyErrors, ...
    "Position", [390 110 555 285]);
setMatlabFunctionScript(applyErrors, gnssRtnFunctionScript());

add_block("simulink/Ports & Subsystems/Out1", parent + "/rMeasured_I_m", ...
    "Port", "1", "Position", [630 148 660 162]);
add_block("simulink/Ports & Subsystems/Out1", parent + "/vMeasured_I_m_s", ...
    "Port", "2", "Position", [630 233 660 247]);

addNamedLine(parent, "rTruth_I_m/1", "Apply RTN Errors/1", "");
addNamedLine(parent, "vTruth_I_m_s/1", "Apply RTN Errors/2", "");
addNamedLine(parent, "Position Error RTN/1", "Apply RTN Errors/3", "");
addNamedLine(parent, "Velocity Error RTN/1", "Apply RTN Errors/4", "");
addNamedLine(parent, "Apply RTN Errors/1", "rMeasured_I_m/1", ...
    "r_measured_I_m");
addNamedLine(parent, "Apply RTN Errors/2", "vMeasured_I_m_s/1", ...
    "v_measured_I_m_s");
end

function buildPositionRtnError(parent)
% Description:
%   Builds the radial ionosphere, periodic, colored, and white position
%   error components in the RTN frame.

clearSubsystemTemplate(parent);

add_block("simulink/Sources/Constant", parent + "/Ionosphere Bias", ...
    "Value", "[AOCS_SensorConfig.GNSS.radial_ionosphere_bias_m; 0; 0]", ...
    "Position", [20 20 105 50]);
addPeriodicError(parent, "Position", 85, ...
    "AOCS_SensorConfig.GNSS.position_periodic_amplitude_RTN_m");

gaussMarkov = parent + "/Gauss-Markov";
add_block("simulink/Ports & Subsystems/Subsystem", gaussMarkov, ...
    "Position", [20 185 205 255]);
buildGaussMarkovModel(gaussMarkov, ...
    "AOCS_SensorConfig.GNSS.position_gauss_markov_seed", ...
    "AOCS_SensorConfig.GNSS.position_gauss_markov_step_std_RTN_m");

add_block("simulink/Sources/Random Number", parent + "/White Noise W", ...
    "Mean", "zeros(3,1)", "Variance", "ones(3,1)", ...
    "Seed", "AOCS_SensorConfig.GNSS.position_white_noise_seed", ...
    "SampleTime", "AOCS_SensorConfig.GNSS.sample_time_s", ...
    "Position", [20 295 75 325]);
add_block("simulink/Math Operations/Gain", ...
    parent + "/White Noise Std", ...
    "Gain", "diag(AOCS_SensorConfig.GNSS.position_white_noise_std_RTN_m)", ...
    "Multiplication", "Matrix(K*u)", ...
    "Position", [120 295 205 325]);

add_block("simulink/Math Operations/Sum", parent + "/Sum Errors", ...
    "Inputs", "++++", "Position", [285 120 315 220]);
add_block("simulink/Ports & Subsystems/Out1", parent + "/positionError_RTN_m", ...
    "Port", "1", "Position", [385 163 415 177]);

addNamedLine(parent, "Ionosphere Bias/1", "Sum Errors/1", ...
    "ionosphere_bias_RTN_m");
addNamedLine(parent, "Position Periodic Amplitude/1", "Sum Errors/2", ...
    "periodic_position_RTN_m");
addNamedLine(parent, "Gauss-Markov/1", "Sum Errors/3", ...
    "colored_position_RTN_m");
addNamedLine(parent, "White Noise W/1", "White Noise Std/1", "");
addNamedLine(parent, "White Noise Std/1", "Sum Errors/4", ...
    "white_position_RTN_m");
addNamedLine(parent, "Sum Errors/1", "positionError_RTN_m/1", ...
    "position_error_RTN_m");
end

function buildVelocityRtnError(parent)
% Description:
%   Builds periodic, colored, and white velocity errors in the RTN frame.

clearSubsystemTemplate(parent);

addPeriodicError(parent, "Velocity", 40, ...
    "AOCS_SensorConfig.GNSS.velocity_periodic_amplitude_RTN_m_s");

gaussMarkov = parent + "/Gauss-Markov";
add_block("simulink/Ports & Subsystems/Subsystem", gaussMarkov, ...
    "Position", [20 140 205 210]);
buildGaussMarkovModel(gaussMarkov, ...
    "AOCS_SensorConfig.GNSS.velocity_gauss_markov_seed", ...
    "AOCS_SensorConfig.GNSS.velocity_gauss_markov_step_std_RTN_m_s");

add_block("simulink/Sources/Random Number", parent + "/White Noise W", ...
    "Mean", "zeros(3,1)", "Variance", "ones(3,1)", ...
    "Seed", "AOCS_SensorConfig.GNSS.velocity_white_noise_seed", ...
    "SampleTime", "AOCS_SensorConfig.GNSS.sample_time_s", ...
    "Position", [20 250 75 280]);
add_block("simulink/Math Operations/Gain", ...
    parent + "/White Noise Std", ...
    "Gain", "diag(AOCS_SensorConfig.GNSS.velocity_white_noise_std_RTN_m_s)", ...
    "Multiplication", "Matrix(K*u)", ...
    "Position", [120 250 205 280]);

add_block("simulink/Math Operations/Sum", parent + "/Sum Errors", ...
    "Inputs", "+++", "Position", [285 90 315 190]);
add_block("simulink/Ports & Subsystems/Out1", parent + "/velocityError_RTN_m_s", ...
    "Port", "1", "Position", [385 133 415 147]);

addNamedLine(parent, "Velocity Periodic Amplitude/1", "Sum Errors/1", ...
    "periodic_velocity_RTN_m_s");
addNamedLine(parent, "Gauss-Markov/1", "Sum Errors/2", ...
    "colored_velocity_RTN_m_s");
addNamedLine(parent, "White Noise W/1", "White Noise Std/1", "");
addNamedLine(parent, "White Noise Std/1", "Sum Errors/3", ...
    "white_velocity_RTN_m_s");
addNamedLine(parent, "Sum Errors/1", "velocityError_RTN_m_s/1", ...
    "velocity_error_RTN_m_s");
end

function addPeriodicError(parent, prefix, y, amplitudeExpression)
% Description:
%   Adds one scalar orbital-phase sine and maps it to an RTN error vector.

add_block("simulink/Sources/Sine Wave", parent + "/Orbital Phase Sine", ...
    "Amplitude", "1", "Bias", "0", ...
    "Frequency", "AOCS_SensorConfig.GNSS.once_per_orbit_angular_rate_rad_s", ...
    "Phase", "AOCS_SensorConfig.GNSS.once_per_orbit_phase_rad", ...
    "SampleTime", "AOCS_SensorConfig.GNSS.sample_time_s", ...
    "Position", [20 y 85 y + 30]);
add_block("simulink/Math Operations/Gain", ...
    parent + "/" + prefix + " Periodic Amplitude", ...
    "Gain", amplitudeExpression, ...
    "Multiplication", "Matrix(K*u)", ...
    "Position", [120 y 205 y + 30]);

addNamedLine(parent, "Orbital Phase Sine/1", ...
    prefix + " Periodic Amplitude/1", "");
end

function buildGaussMarkovModel(parent, seedExpression, stepStdExpression)
% Description:
%   Builds a vector first-order Gauss-Markov recursion at the GNSS sample
%   rate and publishes its current state.

clearSubsystemTemplate(parent);

add_block("simulink/Sources/Random Number", parent + "/Innovation W", ...
    "Mean", "zeros(3,1)", "Variance", "ones(3,1)", ...
    "Seed", seedExpression, ...
    "SampleTime", "AOCS_SensorConfig.GNSS.sample_time_s", ...
    "Position", [20 45 75 75]);
add_block("simulink/Math Operations/Gain", parent + "/Innovation Std", ...
    "Gain", "diag(" + stepStdExpression + ")", ...
    "Multiplication", "Matrix(K*u)", ...
    "Position", [120 45 200 75]);
add_block("simulink/Discrete/Unit Delay", parent + "/State Delay", ...
    "InitialCondition", "zeros(3,1)", ...
    "SampleTime", "AOCS_SensorConfig.GNSS.sample_time_s", ...
    "Position", [120 135 175 165]);
add_block("simulink/Math Operations/Gain", parent + "/State Transition", ...
    "Gain", "AOCS_SensorConfig.GNSS.gauss_markov_alpha", ...
    "Position", [220 135 285 165]);
add_block("simulink/Math Operations/Sum", parent + "/State Update", ...
    "Inputs", "++", "Position", [330 65 360 135]);
add_block("simulink/Ports & Subsystems/Out1", parent + "/state", ...
    "Port", "1", "Position", [435 93 465 107]);

addNamedLine(parent, "Innovation W/1", "Innovation Std/1", "");
addNamedLine(parent, "Innovation Std/1", "State Update/1", "");
addNamedLine(parent, "State Delay/1", "State Transition/1", "");
addNamedLine(parent, "State Transition/1", "State Update/2", "");
addNamedLine(parent, "State Update/1", "State Delay/1", "");
addNamedLine(parent, "State Update/1", "state/1", "state");
end

function buildFixAvailability(parent)
% Description:
%   Builds acquisition delay, configured dropout mode, and stochastic
%   per-sample loss-of-fix logic.

clearSubsystemTemplate(parent);

add_block("simulink/Ports & Subsystems/In1", parent + "/GnssConfig", ...
    "Port", "1", "Position", [20 103 50 117]);
add_block("simulink/Signal Routing/Bus Selector", ...
    parent + "/Select Availability Config", ...
    "OutputSignals", ...
    "mode_id,acquisition_time_s,dropout_probability_per_sample", ...
    "Position", [90 55 95 165]);
add_block("simulink/Sources/Clock", parent + "/Simulation Time", ...
    "Position", [145 20 180 50]);
add_block("simulink/Logic and Bit Operations/Relational Operator", ...
    parent + "/Acquisition Complete", "Operator", ">=", ...
    "Position", [240 25 285 65]);
add_block("simulink/Sources/Uniform Random Number", ...
    parent + "/Dropout Uniform", ...
    "Minimum", "0", "Maximum", "1", ...
    "Seed", "AOCS_SensorConfig.GNSS.dropout_seed", ...
    "SampleTime", "AOCS_SensorConfig.GNSS.sample_time_s", ...
    "Position", [145 105 205 135]);
add_block("simulink/Logic and Bit Operations/Relational Operator", ...
    parent + "/Random Dropout", "Operator", "<", ...
    "Position", [240 100 285 140]);
add_block("simulink/Logic and Bit Operations/Compare To Constant", ...
    parent + "/Configured Dropout", ...
    "relop", "==", "const", "2", ...
    "Position", [145 180 220 210]);
add_block("simulink/Logic and Bit Operations/Logical Operator", ...
    parent + "/No Random Dropout", "Operator", "NOT", ...
    "Position", [335 105 370 135]);
add_block("simulink/Logic and Bit Operations/Logical Operator", ...
    parent + "/No Configured Dropout", "Operator", "NOT", ...
    "Position", [335 180 370 210]);
add_block("simulink/Logic and Bit Operations/Logical Operator", ...
    parent + "/Fix Available", "Operator", "AND", "Inputs", "3", ...
    "Position", [425 80 470 155]);
add_block("simulink/Signal Attributes/Data Type Conversion", ...
    parent + "/To Double", "OutDataTypeStr", "double", ...
    "Position", [520 100 575 130]);
add_block("simulink/Ports & Subsystems/Out1", parent + "/fix_available", ...
    "Port", "1", "Position", [630 108 660 122]);

addNamedLine(parent, "GnssConfig/1", "Select Availability Config/1", "");
addNamedLine(parent, "Simulation Time/1", "Acquisition Complete/1", "");
addNamedLine(parent, "Select Availability Config/2", ...
    "Acquisition Complete/2", "acquisition_time_s");
addNamedLine(parent, "Dropout Uniform/1", "Random Dropout/1", "");
addNamedLine(parent, "Select Availability Config/3", ...
    "Random Dropout/2", "dropout_probability_per_sample");
addNamedLine(parent, "Select Availability Config/1", ...
    "Configured Dropout/1", "mode_id");
addNamedLine(parent, "Random Dropout/1", "No Random Dropout/1", "");
addNamedLine(parent, "Configured Dropout/1", ...
    "No Configured Dropout/1", "");
addNamedLine(parent, "Acquisition Complete/1", "Fix Available/1", "");
addNamedLine(parent, "No Random Dropout/1", "Fix Available/2", "");
addNamedLine(parent, "No Configured Dropout/1", "Fix Available/3", "");
addNamedLine(parent, "Fix Available/1", "To Double/1", "");
addNamedLine(parent, "To Double/1", "fix_available/1", "fix_available");
end

function buildOutputSelection(parent)
% Description:
%   Separates position, velocity, and validity output selection into three
%   compact lanes. Local Goto/From tags distribute shared config and fix
%   availability without long crossing signal lines.

clearSubsystemTemplate(parent);

add_block("simulink/Ports & Subsystems/In1", parent + "/rNominal_I_m", ...
    "Port", "1", "Position", [20 63 50 77]);
add_block("simulink/Ports & Subsystems/In1", parent + "/vNominal_I_m_s", ...
    "Port", "2", "Position", [20 203 50 217]);
add_block("simulink/Ports & Subsystems/In1", parent + "/GnssConfig", ...
    "Port", "3", "Position", [20 363 50 377]);
add_block("simulink/Ports & Subsystems/In1", parent + "/fix_available", ...
    "Port", "4", "Position", [20 433 50 447]);

addLocalGoto(parent, "Goto GNSS Config", "GNSS_CFG", [105 350 195 380]);
addLocalGoto(parent, "Goto Fix Available", "GNSS_FIX", [105 420 195 450]);

positionOutput = parent + "/Position Output";
add_block("simulink/Ports & Subsystems/Subsystem", positionOutput, ...
    "Position", [315 20 585 120]);
buildVectorOutputSelection(positionOutput, "rNominal_I_m", ...
    "failure_r_I_m", "zeros(3,1)", "r_I_m");

velocityOutput = parent + "/Velocity Output";
add_block("simulink/Ports & Subsystems/Subsystem", velocityOutput, ...
    "Position", [315 160 585 260]);
buildVectorOutputSelection(velocityOutput, "vNominal_I_m_s", ...
    "failure_v_I_m_s", "zeros(3,1)", "v_I_m_s");

validityOutput = parent + "/Validity Output";
add_block("simulink/Ports & Subsystems/Subsystem", validityOutput, ...
    "Position", [315 300 585 400]);
buildValidityOutputSelection(validityOutput);

addLocalFrom(parent, "Position Config", "GNSS_CFG", [220 65 280 85]);
addLocalFrom(parent, "Position Fix", "GNSS_FIX", [220 90 280 110]);
addLocalFrom(parent, "Velocity Config", "GNSS_CFG", [220 205 280 225]);
addLocalFrom(parent, "Velocity Fix", "GNSS_FIX", [220 230 280 250]);
addLocalFrom(parent, "Validity Config", "GNSS_CFG", [220 335 280 355]);
addLocalFrom(parent, "Validity Fix", "GNSS_FIX", [220 365 280 385]);

add_block("simulink/Ports & Subsystems/Out1", parent + "/r_I_m", ...
    "Port", "1", "Position", [660 63 690 77]);
add_block("simulink/Ports & Subsystems/Out1", parent + "/v_I_m_s", ...
    "Port", "2", "Position", [660 203 690 217]);
add_block("simulink/Ports & Subsystems/Out1", parent + "/valid", ...
    "Port", "3", "Position", [660 343 690 357]);

addNamedLine(parent, "GnssConfig/1", "Goto GNSS Config/1", "");
addNamedLine(parent, "fix_available/1", "Goto Fix Available/1", "");
addNamedLine(parent, "rNominal_I_m/1", "Position Output/1", "");
addNamedLine(parent, "Position Config/1", "Position Output/2", "");
addNamedLine(parent, "Position Fix/1", "Position Output/3", "");
addNamedLine(parent, "Position Output/1", "r_I_m/1", "r_I_m");
addNamedLine(parent, "vNominal_I_m_s/1", "Velocity Output/1", "");
addNamedLine(parent, "Velocity Config/1", "Velocity Output/2", "");
addNamedLine(parent, "Velocity Fix/1", "Velocity Output/3", "");
addNamedLine(parent, "Velocity Output/1", "v_I_m_s/1", "v_I_m_s");
addNamedLine(parent, "Validity Config/1", "Validity Output/1", "");
addNamedLine(parent, "Validity Fix/1", "Validity Output/2", "");
addNamedLine(parent, "Validity Output/1", "valid/1", "valid");
end

function buildVectorOutputSelection(parent, nominalName, failureField, ...
    zeroExpression, outputName)
% Description:
%   Builds one vector lane for no-fix, failure, and enabled selection.

clearSubsystemTemplate(parent);

add_block("simulink/Ports & Subsystems/In1", parent + "/" + nominalName, ...
    "Port", "1", "Position", [20 33 50 47]);
add_block("simulink/Ports & Subsystems/In1", parent + "/GnssConfig", ...
    "Port", "2", "Position", [20 248 50 262]);
add_block("simulink/Ports & Subsystems/In1", parent + "/fix_available", ...
    "Port", "3", "Position", [20 83 50 97]);

add_block("simulink/Sources/Constant", parent + "/No Fix Output", ...
    "Value", zeroExpression, "Position", [105 105 150 135]);
addModeSwitch(parent, "Select Fix", [200 30 250 120]);
addLocalFrom(parent, "Failure Value", "FAIL_VALUE", [310 35 370 55]);
addLocalFrom(parent, "Failure Mode", "IS_FAILURE", [310 75 370 95]);
addModeSwitch(parent, "Select Failure", [400 30 450 120]);
addLocalFrom(parent, "Enabled", "ENABLED", [510 72 565 92]);
add_block("simulink/Sources/Constant", parent + "/Disabled Output", ...
    "Value", zeroExpression, "Position", [510 105 555 135]);
addModeSwitch(parent, "Select Enabled", [600 30 650 120]);

add_block("simulink/Signal Routing/Bus Selector", ...
    parent + "/Select GNSS Config", ...
    "OutputSignals", strjoin(["enabled", "mode_id", failureField], ","), ...
    "Position", [95 200 100 300]);
addLocalGoto(parent, "Goto Enabled", "ENABLED", [180 205 260 225]);
add_block("simulink/Logic and Bit Operations/Compare To Constant", ...
    parent + "/Is Failure", ...
    "relop", "==", "const", "3", ...
    "Position", [180 240 245 270]);
addLocalGoto(parent, "Goto Failure Mode", "IS_FAILURE", ...
    [290 245 390 265]);
addLocalGoto(parent, "Goto Failure Value", "FAIL_VALUE", ...
    [180 280 280 300]);

add_block("simulink/Ports & Subsystems/Out1", parent + "/" + outputName, ...
    "Port", "1", "Position", [720 68 750 82]);

addNamedLine(parent, nominalName + "/1", "Select Fix/1", "");
addNamedLine(parent, "fix_available/1", "Select Fix/2", "");
addNamedLine(parent, "No Fix Output/1", "Select Fix/3", "");
addNamedLine(parent, "Failure Value/1", "Select Failure/1", "");
addNamedLine(parent, "Failure Mode/1", "Select Failure/2", "");
addNamedLine(parent, "Select Fix/1", "Select Failure/3", "");
addNamedLine(parent, "Select Failure/1", "Select Enabled/1", "");
addNamedLine(parent, "Enabled/1", "Select Enabled/2", "");
addNamedLine(parent, "Disabled Output/1", "Select Enabled/3", "");
addNamedLine(parent, "Select Enabled/1", outputName + "/1", outputName);

addNamedLine(parent, "GnssConfig/1", "Select GNSS Config/1", "");
addNamedLine(parent, "Select GNSS Config/1", "Goto Enabled/1", "enabled");
addNamedLine(parent, "Select GNSS Config/2", "Is Failure/1", "mode_id");
addNamedLine(parent, "Is Failure/1", "Goto Failure Mode/1", "");
addNamedLine(parent, "Select GNSS Config/3", ...
    "Goto Failure Value/1", failureField);
end

function buildValidityOutputSelection(parent)
% Description:
%   Builds scalar navigation-fix validity selection for all receiver modes.

clearSubsystemTemplate(parent);

add_block("simulink/Ports & Subsystems/In1", parent + "/GnssConfig", ...
    "Port", "1", "Position", [20 248 50 262]);
add_block("simulink/Ports & Subsystems/In1", parent + "/fix_available", ...
    "Port", "2", "Position", [20 83 50 97]);

add_block("simulink/Sources/Constant", parent + "/Nominal Valid", ...
    "Value", "1", "Position", [20 30 65 60]);
add_block("simulink/Sources/Constant", parent + "/No Fix Invalid", ...
    "Value", "0", "Position", [105 105 150 135]);
addModeSwitch(parent, "Select Fix", [200 30 250 120]);
addLocalFrom(parent, "Failure Value", "FAIL_VALUE", [310 35 370 55]);
addLocalFrom(parent, "Failure Mode", "IS_FAILURE", [310 75 370 95]);
addModeSwitch(parent, "Select Failure", [400 30 450 120]);
addLocalFrom(parent, "Enabled", "ENABLED", [510 72 565 92]);
add_block("simulink/Sources/Constant", parent + "/Disabled Invalid", ...
    "Value", "0", "Position", [510 105 555 135]);
addModeSwitch(parent, "Select Enabled", [600 30 650 120]);

add_block("simulink/Signal Routing/Bus Selector", ...
    parent + "/Select GNSS Config", ...
    "OutputSignals", "enabled,mode_id,failure_valid", ...
    "Position", [95 200 100 300]);
addLocalGoto(parent, "Goto Enabled", "ENABLED", [180 205 260 225]);
add_block("simulink/Logic and Bit Operations/Compare To Constant", ...
    parent + "/Is Failure", ...
    "relop", "==", "const", "3", ...
    "Position", [180 240 245 270]);
addLocalGoto(parent, "Goto Failure Mode", "IS_FAILURE", ...
    [290 245 390 265]);
addLocalGoto(parent, "Goto Failure Value", "FAIL_VALUE", ...
    [180 280 280 300]);

add_block("simulink/Ports & Subsystems/Out1", parent + "/valid", ...
    "Port", "1", "Position", [720 68 750 82]);

addNamedLine(parent, "Nominal Valid/1", "Select Fix/1", "");
addNamedLine(parent, "fix_available/1", "Select Fix/2", "");
addNamedLine(parent, "No Fix Invalid/1", "Select Fix/3", "");
addNamedLine(parent, "Failure Value/1", "Select Failure/1", "");
addNamedLine(parent, "Failure Mode/1", "Select Failure/2", "");
addNamedLine(parent, "Select Fix/1", "Select Failure/3", "");
addNamedLine(parent, "Select Failure/1", "Select Enabled/1", "");
addNamedLine(parent, "Enabled/1", "Select Enabled/2", "");
addNamedLine(parent, "Disabled Invalid/1", "Select Enabled/3", "");
addNamedLine(parent, "Select Enabled/1", "valid/1", "valid");

addNamedLine(parent, "GnssConfig/1", "Select GNSS Config/1", "");
addNamedLine(parent, "Select GNSS Config/1", "Goto Enabled/1", "enabled");
addNamedLine(parent, "Select GNSS Config/2", "Is Failure/1", "mode_id");
addNamedLine(parent, "Is Failure/1", "Goto Failure Mode/1", "");
addNamedLine(parent, "Select GNSS Config/3", ...
    "Goto Failure Value/1", "failure_valid");
end

function addLocalGoto(parent, name, tag, position)
% Description:
%   Adds a locally scoped Goto block for short, contained signal routing.

add_block("simulink/Signal Routing/Goto", parent + "/" + name, ...
    "GotoTag", tag, "TagVisibility", "local", "Position", position);
end

function addLocalFrom(parent, name, tag, position)
% Description:
%   Adds a From block connected to a locally scoped Goto tag.

add_block("simulink/Signal Routing/From", parent + "/" + name, ...
    "GotoTag", tag, "ShowName", "off", "Position", position);
end

function addModeSwitch(parent, name, position)
% Description:
%   Adds a switch controlled by a scalar boolean mode signal.

add_block("simulink/Signal Routing/Switch", parent + "/" + name, ...
    "Criteria", "u2 > Threshold", ...
    "Threshold", "0.5", ...
    "Position", position);
end

function setMatlabFunctionScript(block, script)
% Description:
%   Replaces the script of a MATLAB Function block.

root = sfroot();
blockName = string(get_param(block, "Name"));
chart = root.find("-isa", "Stateflow.EMChart", "Path", char(block), ...
    "Name", char(blockName));
if isempty(chart)
    error("AOCS:GNSS:MissingChart", ...
        "MATLAB Function chart not found: %s", block);
end
chart.Script = script;
end

function script = gnssRtnFunctionScript()
% Description:
%   Returns the MATLAB Function script that maps RTN PVT errors to the
%   inertial output frame.

script = strjoin([ ...
    "function [rMeasured_I_m, vMeasured_I_m_s] = applyRtnErrors(" + ...
        "rTruth_I_m, vTruth_I_m_s, positionError_RTN_m, velocityError_RTN_m_s)" ...
    "%#codegen" ...
    "[rMeasured_I_m, vMeasured_I_m_s] = applyGnssRtnErrors(" + ...
        "rTruth_I_m, vTruth_I_m_s, positionError_RTN_m, velocityError_RTN_m_s);" ...
    "end"], newline);
end

function connectOrbitStateToGnss(modelName)
% Description:
%   Appends OrbitState to the plant-state selector and feeds the GNSS model.

parent = modelName + "/Sensors";
selector = firstBlockByType(parent, "BusSelector");
signals = strtrim(split(string(get_param(selector, "OutputSignals")), ","));
if ~any(signals == "OrbitState")
    signals(end + 1) = "OrbitState";
    set_param(selector, "OutputSignals", strjoin(signals, ","));
end
orbitStatePort = find(signals == "OrbitState", 1);

addNamedLine(parent, blockPortSpec(selector, orbitStatePort), ...
    "GNSS/1", "OrbitState");
end

function publishSensorMeasurements(modelName)
% Description:
%   Adds GNSS measurements to the top-level SensorMeasurementBus.

parent = modelName + "/Sensors";
assembly = parent + "/Sensor Measurement Bus Assembly";

if getSimulinkBlockHandle(assembly) <= 0
    error("AOCS:Sensors:MissingBlock", ...
        "Missing sensor measurement bus assembly: %s.", assembly);
end

set_param(assembly, ...
    "Inputs", "4", ...
    "OutDataTypeStr", "Bus: SensorMeasurementBus", ...
    "Position", [565 74 570 570]);
addNamedLine(parent, "GNSS/1", ...
    "Sensor Measurement Bus Assembly/4", "GNSS");
end

function block = firstBlockByType(parent, blockType)
% Description:
%   Returns the first direct child block with the requested BlockType.

matches = find_system(parent, "SearchDepth", 1, "BlockType", blockType);
if isempty(matches)
    error("AOCS:Sensors:MissingBlock", ...
        "No %s block found under %s.", blockType, parent);
end
block = string(matches{1});
end

function spec = blockPortSpec(block, portNumber)
% Description:
%   Builds a block/port string relative to the block parent for add_line.

[~, name] = fileparts(char(block));
spec = string(name) + "/" + string(portNumber);
end

function addNamedLine(parent, sourcePort, destinationPort, signalName)
% Description:
%   Adds a line and assigns an explicit signal name when possible.

deleteDestinationLine(parent, destinationPort);
line = add_line(parent, sourcePort, destinationPort, "autorouting", "on");
if strlength(string(signalName)) == 0
    return;
end

try
    set_param(line, "Name", char(signalName));
catch exception
    if ~contains(exception.message, "Bus Selector")
        rethrow(exception);
    end
end
end

function deleteDestinationLine(parent, destinationPort)
% Description:
%   Deletes any existing line attached to a destination port string.

parts = split(string(destinationPort), "/");
if numel(parts) < 2
    return;
end

inputIndex = str2double(parts(end));
if isnan(inputIndex)
    return;
end

block = parent + "/" + strjoin(parts(1:end - 1), "/");
deleteInputLine(block, inputIndex);
end

function deleteInputLine(block, inputIndex)
% Description:
%   Deletes the line attached to a specific block input port, if present.

if getSimulinkBlockHandle(block) <= 0
    return;
end

ports = get_param(block, "PortHandles");
if numel(ports.Inport) < inputIndex
    return;
end

line = get_param(ports.Inport(inputIndex), "Line");
if line > 0
    delete_line(line);
end
end

function deleteBlockIfExists(block)
% Description:
%   Deletes a block when it exists.

if getSimulinkBlockHandle(block) > 0
    delete_block(block);
end
end

function clearSubsystemTemplate(parent)
% Description:
%   Removes the default In1, Out1, and connecting line from a new subsystem.

lines = find_system(parent, "SearchDepth", 1, "FindAll", "on", ...
    "Type", "line");
for index = 1:numel(lines)
    delete_line(lines(index));
end

deleteBlockIfExists(parent + "/In1");
deleteBlockIfExists(parent + "/Out1");
end

function closeIfLoaded(modelName)
% Description:
%   Closes a loaded model without saving additional changes.

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end
