classdef AttitudeInitializationTest < matlab.unittest.TestCase
	% Description:
	%   Verifies navigation-fed reference-vector configuration, TRIAD geometry,
	%   bus contracts, and the Sensors -> Drivers -> GNC wiring boundary.

	properties (Constant)
		ModelName = "aocs_plant"
	end

	properties
		ProjectRoot
	end

	methods (TestClassSetup)
		function prepareAttitudeInitializationEnvironment(testCase)
			% Description:
			%   Locates the project root and installs production/test paths.
			%
			% Arguments:
			%   testCase - matlab.unittest.TestCase instance.
			%
			% Outputs:
			%   None.

			testCase.ProjectRoot = fileparts(fileparts(fileparts( ...
				mfilename("fullpath"))));
			setupAocsPaths(testCase.ProjectRoot, true);
		end
	end

	methods (TestMethodTeardown)
		function closeModels(testCase)
			% Description:
			%   Closes the production model and any temporary TRIAD harness.
			%
			% Arguments:
			%   testCase - matlab.unittest.TestCase instance.
			%
			% Outputs:
			%   None.

			if bdIsLoaded(testCase.ModelName)
				close_system(testCase.ModelName, 0);
			end
			harnesses = find_system("Regexp", "on", ...
				"Type", "block_diagram", "Name", "triad_test_.*");
			for index = 1:numel(harnesses)
				close_system(harnesses{index}, 0);
			end
		end
	end

	methods (Test, TestTags = "Configuration")
		function configurationLoadsSgp4AndTriadSettings(testCase)
			% Description:
			%   Confirms JSON settings and derived mission constants are
			%   published through one typed GNC configuration parameter.
			%
			% Arguments:
			%   testCase - matlab.unittest.TestCase instance.
			%
			% Outputs:
			%   None.

			[AOCS, variables] = setupAocsSimulation("", false);
			initialization = AOCS.GNC.AttitudeInitialization;
			busConfig = AOCS.GNCConfig.AttitudeInitialization;

			testCase.verifyTrue(initialization.Enabled);
			testCase.verifyEqual(AOCS.GNC.SampleTime_s, 0.1);
			testCase.verifyEqual(AOCS.GNCConfig.sample_time_s, 0.1);
			testCase.verifyEqual(initialization.Sgp4SampleTime_s, 1.0);
			testCase.verifyEqual(initialization.Sgp4BStar, 0.0);
			testCase.verifyEqual(initialization.MagnetometerMaxAge_s, 0.25);
			testCase.verifyEqual(initialization.CssMaxAge_s, 0.25);
			testCase.verifyEqual(initialization.MinimumTriadCrossNorm, 0.05);
			testCase.verifyGreaterThan(busConfig.epoch_utc_jd, 2400000.0);
			testCase.verifyEqual(busConfig.sgp4_semi_major_axis_m, 6871000.0);
			testCase.verifyEqual(string(variables.AOCS_GNCConfig.DataType), ...
				"Bus: GNCConfigBus");
			testCase.verifyEqual(variables.AOCS_GNCConfig.Value, AOCS.GNCConfig);
		end
	end

	methods (Test, TestTags = "Contracts")
		function contractsDescribeReferencesAndInitialization(testCase)
			% Description:
			%   Verifies stable reference-vector and TRIAD result buses.
			%
			% Arguments:
			%   testCase - matlab.unittest.TestCase instance.
			%
			% Outputs:
			%   None.

			createAocsBuses();
			references = evalin("base", "ReferenceVectorBus");
			initialization = evalin("base", "AttitudeInitializationBus");
			sgp4 = evalin("base", "Sgp4ConfigBus");
			gnc = evalin("base", "GNCConfigBus");

			testCase.verifyEqual(string({references.Elements.Name}), [ ...
				"sun_I_unit", "B_I_T", "valid", "orbit_update_time_s", ...
				"orbit_solution_id"]);
			testCase.verifyEqual(string(references.Elements(5).DataType), "uint32");
			testCase.verifyEqual(string({initialization.Elements.Name}), [ ...
				"q_BI", "DCM_BI", "valid", "solution_time_s", ...
				"magnetometer_sequence_id", "css_sequence_id", ...
				"orbit_solution_id"]);
			testCase.verifyEqual(initialization.Elements(1).Dimensions, [4 1]);
			testCase.verifyEqual(initialization.Elements(2).Dimensions, [3 3]);
			testCase.verifyEqual(string({sgp4.Elements.Name}), [ ...
				"epoch_utc_jd", "sgp4_semi_major_axis_m", ...
				"sgp4_eccentricity", "sgp4_inclination_deg", ...
				"sgp4_raan_deg", "sgp4_argument_of_periapsis_deg", ...
				"sgp4_true_anomaly_deg", "sgp4_bstar"]);
			testCase.verifyEqual(string({sgp4.Elements.DataType}), ...
				repmat("double", 1, 8));
			testCase.verifyEqual(string({gnc.Elements.Name}), ...
				["sample_time_s", "AttitudeInitialization", "MEKF", ...
				"AttitudeHealth"]);
		end
	end

	methods (Test, TestTags = "Algorithms")
		function sgp4ProducesFiniteMovingOrbit(testCase)
			% Description:
			%   Confirms the onboard adapter returns finite LEO states and
			%   advances position over one minute without using GNSS.
			%
			% Arguments:
			%   testCase - matlab.unittest.TestCase instance.
			%
			% Outputs:
			%   None.

			AOCS = loadAocsSimulationConfig();
			config = AOCS.GNCConfig.AttitudeInitialization;
			input0 = testCase.sgp4Input(config, 0.0);
			input60 = testCase.sgp4Input(config, 60.0);
			state0 = propagateOnboardSgp4(input0);
			state60 = propagateOnboardSgp4(input60);

			testCase.verifyEqual(state0(7), 1.0);
			testCase.verifyEqual(state60(7), 1.0);
			testCase.verifyTrue(all(isfinite(state60)));
			testCase.verifyGreaterThan(norm(state0(1:3)), 6.0e6);
			testCase.verifyLessThan(norm(state0(1:3)), 8.0e6);
			testCase.verifyGreaterThan(norm(state60(1:3) - state0(1:3)), 1.0e5);
		end

		function triadRecoversKnownAttitude(testCase)
			% Description:
			%   Exercises the block implementation with a known 90-degree
			%   inertial-to-body rotation and two orthogonal vectors.
			%
			% Arguments:
			%   testCase - matlab.unittest.TestCase instance.
			%
			% Outputs:
			%   None.

			C_BI = [0 1 0; -1 0 0; 0 0 1];
			sun_I = [1; 0; 0];
			magnetic_I = [0; 1; 0];
			result = testCase.simulateTriad(C_BI * sun_I, ...
				C_BI * magnetic_I, sun_I, magnetic_I);

			testCase.verifyEqual(result.valid, 1.0);
			testCase.verifyEqual(result.DCM_BI, C_BI, "AbsTol", 1.0e-12);

			expectedQuaternion = [sqrt(0.5); 0; 0; sqrt(0.5)];

			testCase.verifyEqual(abs(result.q_BI.' * expectedQuaternion), ...
				1.0, "AbsTol", 1.0e-12);
		end

		function triadRejectsCollinearVectors(testCase)
			% Description:
			%   Confirms TRIAD reports invalid geometry and returns safe
			%   identity outputs when either vector pair is collinear.
			%
			% Arguments:
			%   testCase - matlab.unittest.TestCase instance.
			%
			% Outputs:
			%   None.

			vector = [1; 0; 0];
			result = testCase.simulateTriad(vector, vector, vector, vector);

			testCase.verifyEqual(result.valid, 0.0);
			testCase.verifyEqual(result.DCM_BI, eye(3), "AbsTol", 1.0e-12);
			testCase.verifyEqual(result.q_BI, [1; 0; 0; 0], ...
				"AbsTol", 1.0e-12);
		end
	end

	methods (Test, TestTags = ["Wiring", "FullPlant"])
		function wiringUsesCommonNavigationWithoutPlantTruth(testCase)
			% Confirms ordered navigation-fed references and TRIAD wiring without plant truth.

			setupAocsSimulation;
			load_system(fullfile(testCase.ProjectRoot, "models", ...
				testCase.ModelName + ".slx"));
			gnc = testCase.ModelName + "/GNC";
			references = gnc + "/Onboard Reference Vectors";
			estimator = gnc + "/State Estimation (MEKF)";

			gncInports = find_system(gnc, "SearchDepth", 1, "BlockType", "Inport");
			referenceInports = find_system(references, ...
				"SearchDepth", 1, "BlockType", "Inport");

			testCase.verifyEmpty(gncInports);
			testCase.verifyNumElements(referenceInports, 3);
			testCase.verifyNumElements(find_system(references, ...
				"SearchDepth", 1, "BlockType", "Outport"), 3);
			testCase.verifyEqual(string(get_param(references + "/NavigationState", ...
				"OutDataTypeStr")), "Bus: OrbitNavigationStateBus");
			referencePorts = get_param(references, "PortHandles");
			inputNames = ["NavigationState", "NavigationTime"];
			sourceNames = ["Orbit Determination & Navigation", "Navigation_Time_10Hz"];
			for index = 1:numel(inputNames)
				port = str2double(get_param(references + "/" + inputNames(index), "Port"));
				line = get_param(referencePorts.Inport(port), "Line");
				source = get_param(line, "SrcBlockHandle");
				testCase.verifyEqual(string(get_param(source, "Name")), sourceNames(index));
			end
			testCase.verifyEmpty(find_system(references, "Regexp", "on", ...
				"Name", ".*GNSS.*"));
			testCase.verifyEmpty(find_system(references, ...
				"Name", "Onboard Orbit Propagator (SGP4)"));
			testCase.verifyEmpty(find_system(references, ...
				"Name", "SGP4 Config Bus Assembly"));
			testCase.verifyEqual(string(get_param(references + ...
				"/NavigationUsable", "Port")), "3");
			for model = ["Magnetic Reference Model", "Sun Reference Model"]
				testCase.verifyNumElements(find_system(references + "/" + model, ...
					"SearchDepth", 1, "BlockType", "EnablePort"), 1);
			end
			guidance = gnc + "/Guidance";
			testCase.verifyNumElements(find_system(guidance, ...
				"SearchDepth", 1, "BlockType", "Inport"), 1);
			testCase.verifyEmpty(find_system(guidance, ...
				"SearchDepth", 1, "Name", "NavigationTime"));
			testCase.verifyEqual(string(get_param(guidance + ...
				"/Select Navigation State", "OutputSignals")), ...
				"position_I_m,velocity_I_m_s,valid");
			testCase.verifyEmpty(find_system(references, ...
				"Name", "Lvlh Pointing Guidance"));
			testCase.verifyNumElements(find_system(guidance + ...
				"/LVLH", "SearchDepth", 1, ...
				"BlockType", "EnablePort"), 1);
			testCase.verifyNumElements(find_system(guidance, ...
				"SearchDepth", 1, "BlockType", "TriggerPort"), 1);
			splitPorts = get_param(gnc + "/GNC Execution Order", "PortHandles");
			testCase.verifyNumElements(splitPorts.Outport, 7);
			expectedOrder = ["Sensor Access", ...
				"Orbit Determination & Navigation", "Onboard Reference Vectors", ...
				"TRIAD Initialization", "State Estimation (MEKF)", ...
				"Guidance", "Attitude Control"];
			for index = 1:7
				line = get_param(splitPorts.Outport(index), "Line");
				destination = get_param(line, "DstBlockHandle");
				testCase.verifyNumElements(destination, 1);
				testCase.verifyEqual(string(get_param(destination, "Name")), ...
					expectedOrder(index));
			end
			controlPorts = get_param(gnc + "/Attitude Control", "PortHandles");
			for name = ["QuaternionReference", "OmegaReference", "AlphaReference"]
				index = str2double(get_param(gnc + "/Attitude Control/" + name, "Port"));
				line = get_param(controlPorts.Inport(index), "Line");
				source = get_param(line, "SrcBlockHandle");
				testCase.verifyEqual(string(getfullname(source)), guidance);
			end
			guidancePorts = get_param(guidance, "PortHandles");
			navigationLine = get_param(guidancePorts.Inport(1), "Line");
			navigationSource = get_param(navigationLine, "SrcBlockHandle");
			testCase.verifyEqual(string(get_param(navigationSource, "Name")), ...
				"Orbit Determination & Navigation");

			selectors = find_system(references, "SearchDepth", 1, ...
				"BlockType", "BusSelector");
			selectorNames = sort(string(get_param(selectors, "Name")));

			testCase.verifyEqual(selectorNames, sort([ ...
				"Select Reference Settings"; "Select Navigation State"]));
			testCase.verifyEqual(string(get_param( ...
				references + "/Select Navigation State", "OutputSignals")), ...
				strjoin(["position_I_m", "velocity_I_m_s", "valid", ...
				"epoch_s", "solution_id"], ","));
			testCase.verifyEqual(string(get_param( ...
				references + "/Select Reference Settings", ...
				"OutputSignals")), strjoin(["minimum_vector_norm", ...
				"enabled", "epoch_decimal_year", "mu_m3_s2", ...
				"epoch_utc", "epoch_tdb_jd", "delta_at_s", ...
				"delta_ut1_s", "polar_motion_rad", "d_cip_rad"], ","));

			testCase.verifyEmpty(find_system(gnc, "Regexp", "on", ...
				"Name", ".*PlantState.*"));
			testCase.verifyNumElements(find_system(estimator, ...
				"SearchDepth", 1, "BlockType", "Inport"), 4);

			estimateOutports = find_system(estimator, ...
				"SearchDepth", 1, "BlockType", "Outport");

			testCase.verifyNumElements(estimateOutports, 2);
			testCase.verifyEqual(string(get_param(estimator + "/AttitudeEstimate", ...
				"OutDataTypeStr")), "Bus: AttitudeEstimateBus");
			testCase.verifyEqual(string(get_param(estimator + "/AttitudeHealth", ...
				"OutDataTypeStr")), "Bus: AttitudeHealthBus");

			charts = find(sfroot, "-isa", "Stateflow.EMChart");
			paths = string(arrayfun(@(chart) chart.Path, charts, ...
				"UniformOutput", false));

			testCase.verifyFalse(any(startsWith(paths, ...
				gnc + "/TRIAD Initialization/TRIAD Attitude Solution")));

			set_param(testCase.ModelName, "SimulationCommand", "update");
		end
	end

	methods (Access = private)
		function input = sgp4Input(~, config, time_s)
			% Description:
			%   Packs the numeric interface expected by propagateOnboardSgp4.
			%
			% Arguments:
			%   config - Attitude-initialization bus payload.
			%   time_s - Elapsed onboard time.
			%
			% Outputs:
			%   input - Nine-element SGP4 adapter input vector.

			input = [time_s; config.epoch_utc_jd; ...
				config.sgp4_semi_major_axis_m; config.sgp4_eccentricity; ...
				config.sgp4_inclination_deg; config.sgp4_raan_deg; ...
				config.sgp4_argument_of_periapsis_deg; ...
				config.sgp4_true_anomaly_deg; config.sgp4_bstar];
		end

		function result = simulateTriad(testCase, sun_B, magnetic_B, ...
				sun_I, magnetic_I)
			% Description:
			%   Copies the production block-only TRIAD core into a temporary
			%   harness and simulates one known vector pair.
			%
			% Arguments:
			%   testCase - matlab.unittest.TestCase instance.
			%   sun_B - Sun unit vector expressed in body axes.
			%   magnetic_B - Magnetic vector expressed in body axes.
			%   sun_I - Sun unit vector expressed in inertial axes.
			%   magnetic_I - Magnetic vector expressed in inertial axes.
			%
			% Outputs:
			%   result - Struct containing q_BI, DCM_BI, and valid.

			setupAocsSimulation;
			load_system(fullfile(testCase.ProjectRoot, "models", ...
				testCase.ModelName + ".slx"));
			name = "triad_test_" + string(randi(1.0e8));
			new_system(name);
			source = testCase.ModelName + ...
				"/GNC/TRIAD Initialization/TRIAD Attitude Solution";

			add_block(source, name + "/TRIAD", ...
				"Position", [260 55 550 405]);

			values = {sun_B, magnetic_B, sun_I, magnetic_I, 1.0, 1.0e-12, 0.05};
			for index = 1:numel(values)
				block = name + "/Input" + index;

				add_block("simulink/Sources/Constant", block, ...
					"Value", mat2str(values{index}, 17), ...
					"Position", [25 35+50*(index-1) 135 55+50*(index-1)]);
				add_line(name, "Input" + index + "/1", ...
					"TRIAD/" + index, "autorouting", "on");
			end

			variableNames = ["triad_q", "triad_dcm", "triad_valid"];
			for index = 1:numel(variableNames)
				block = name + "/Output" + index;

				add_block("simulink/Sinks/To Workspace", block, ...
					"VariableName", variableNames(index), ...
					"SaveFormat", "Timeseries", ...
					"Position", [650 145+75*(index-1) 750 175+75*(index-1)]);
				add_line(name, "TRIAD/" + index, "Output" + index + "/1");
			end
			set_param(name, "Solver", "FixedStepDiscrete", ...
				"FixedStep", "0.1", "StopTime", "0.1");

			output = sim(name);

			q = squeeze(output.get("triad_q").Data);
			dcm = output.get("triad_dcm").Data;
			valid = output.get("triad_valid").Data;
			if size(q, 1) == 4
				result.q_BI = q(:, end);
			else
				result.q_BI = q(end, :).';
			end
			result.DCM_BI = squeeze(dcm(:, :, end));
			result.valid = valid(end);
		end
	end
end
