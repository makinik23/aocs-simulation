classdef Dtm2020AtmosphereHarnessTest < matlab.unittest.TestCase

    properties (SetAccess = private)
        ProjectRoot
        Owner
        HarnessName = "Dtm2020AtmosphereHarness"
        AOCS
    end

    methods (TestClassSetup)
        function prepareModelAndHarness(testCase)
            testCase.ProjectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.Owner = ...
                "aocs_plant/Orbit & Environment/Environment Products/" + ...
                "Atmosphere Products/Atmosphere Model/Atmosphere Products";

            addpath(testCase.ProjectRoot);
            addpath(fullfile(testCase.ProjectRoot, "src", "config"));
            addpath(fullfile(testCase.ProjectRoot, "src", "simulink"));
            addpath(fullfile(testCase.ProjectRoot, "src", "environment"));
            addpath(fullfile(testCase.ProjectRoot, "tests", "harnesses"));

            testCase.AOCS = setupAocsSimulation(fullfile( ...
                testCase.ProjectRoot, "config", "AocsSimulationConfig.json"));
            
            load_system(testCase.AOCS.Model.File);
            applyAocsSimulationSettings(testCase.AOCS.Model.Name, testCase.AOCS);

            info = sltest.harness.find(testCase.Owner, "Name", testCase.HarnessName);
            testCase.assertNotEmpty(info, "DTM2020 atmosphere harness is not registered.");

            sltest.harness.open(testCase.Owner, testCase.HarnessName);
        end
    end

    methods (TestClassTeardown)
        function closeModelAndHarness(testCase)
            try
                sltest.harness.close( ...
                    testCase.Owner, testCase.HarnessName);
            catch
            end

            if bdIsLoaded("aocs_plant")
                close_system("aocs_plant", 0);
            end
        end
    end

    methods (Test)
        function harnessHasExpectedInterface(testCase)
            inputs = rootPortNames(testCase.HarnessName, "Inport");
            outputs = rootPortNames(testCase.HarnessName, "Outport");

            expectedInputs = [
                "r_I_m"
                "v_I_m_s"
                "lla"
                "EnvironmentConfig"
                "EnvironmentContext_decimal_year"
                "EnvironmentContext_mu_m3_s2"
                "EnvironmentContext_epoch_utc"
                "EnvironmentContext_t_s"
                "EnvironmentContext_epoch_tdb_jd"
                "EnvironmentContext_delta_at_s"
                "EnvironmentContext_delta_ut1_s"
                "EnvironmentContext_polar_motion_rad"
                "EnvironmentContext_d_cip_rad"
            ];

            expectedOutputs = [
                "rho_kg_m3"
                "rho_raw_kg_m3"
                "rho_uncertainty_1sigma_kg_m3"
                "T_local_K"
                "T_exo_K"
                "n_O_m3"
                "n_N2_m3"
                "n_O2_m3"
                "n_He_m3"
                "n_H_m3"
                "n_N_m3"
                "v_atm_I_m_s"
            ];

            testCase.verifyEqual(inputs, expectedInputs);
            testCase.verifyEqual(outputs, expectedOutputs);
        end

        function nominalCaseRunsThroughSFunction(testCase)
            time_s = [0; 1];
            sampleCount = numel(time_s);
        
            earthRadius_m = testCase.AOCS.Orbit.CentralBodyConstants.radius_m;
        
            r_I_m = repmat([earthRadius_m + 400e3, 0, 0], sampleCount, 1);
        
            v_I_m_s = repmat([0, 7670, 0], sampleCount, 1);
            lla = repmat([0, 0, 400e3], sampleCount, 1);
        
            inputDataset = Simulink.SimulationData.Dataset();
            inputDataset = addInput(inputDataset, "r_I_m", r_I_m, time_s);
            inputDataset = addInput(inputDataset, "v_I_m_s", v_I_m_s, time_s);
            inputDataset = addInput(inputDataset, "lla", lla, time_s);
        
            configBus = environmentConfigTimeseries(testCase.AOCS.EnvironmentConfig, time_s);
            inputDataset = inputDataset.addElement(configBus, "EnvironmentConfig");
        
            inputDataset = addEnvironmentContext(inputDataset, testCase.AOCS, time_s);
        
            simIn = Simulink.SimulationInput(testCase.HarnessName);
            simIn = simIn.setExternalInput(inputDataset);
            simIn = simIn.setModelParameter( ...
                "StartTime", "0", ...
                "StopTime", "1", ...
                "SolverType", "Fixed-step", ...
                "Solver", "FixedStepDiscrete", ...
                "FixedStep", "1", ...
                "SaveOutput", "on", ...
                "OutputSaveName", "yout", ...
                "SaveFormat", "Dataset");
        
            simOut = sim(simIn);
            yout = simOut.get("yout");
        
            testCase.verifyEqual(yout.numElements, 12);
        
            rho = squeeze(yout{1}.Values.Data);
            rhoRaw = squeeze(yout{2}.Values.Data);
            uncertainty = squeeze(yout{3}.Values.Data);
            TLocal = squeeze(yout{4}.Values.Data);
            TExo = squeeze(yout{5}.Values.Data);
        
            testCase.verifyGreaterThan(rho, zeros(size(rho)));
            testCase.verifyEqual(rho, rhoRaw, "RelTol", 1e-12);
            testCase.verifyGreaterThan(uncertainty, zeros(size(uncertainty)));
            testCase.verifyGreaterThan(TLocal, zeros(size(TLocal)));
            testCase.verifyGreaterThanOrEqual(TExo, TLocal);
        end

        function pipelineMatchesFrozenCnesReference(testCase)
            time_s = [0;1];
            N = numel(time_s);
            benchmarkAOCS = testCase.AOCS;
            
            % 2025-06-29 12:00 UTC:
            % day_of_year = 180.5, local solar time = pi at longitude 0.
            benchmarkAOCS.Epoch.Utc = [2025; 6; 29; 12; 0; 0];
            
            config = benchmarkAOCS.EnvironmentConfig;
            config.atmosphere_enabled = 1;
            config.atmosphere_mode_id = 1;
            config.atmosphere_uncertainty_enabled = 1;
            config.rho_scale_factor = 1.0;
            config.f10_7_sfu = 80.0;
            config.f10_7_81d_sfu = 80.0;
            config.kp = 3.0;
            
            benchmarkAOCS.EnvironmentConfig = config;
            
            radius_m = benchmarkAOCS.Orbit.CentralBodyConstants.radius_m + 300e3;
            
            r_I_m = repmat([radius_m; 0; 0], 1, N);
            v_I_m_s = zeros(3, N);
            lla = repmat([0; 0; 300e3], 1, N);

            inputDataset = Simulink.SimulationData.Dataset();
            inputDataset = addInput(inputDataset, "r_I_m", r_I_m, time_s);
            inputDataset = addInput(inputDataset, "v_I_m_s", v_I_m_s, time_s);
            inputDataset = addInput(inputDataset, "lla", lla, time_s);

            configBus = environmentConfigTimeseries(benchmarkAOCS.EnvironmentConfig, time_s);
            inputDataset = inputDataset.addElement(configBus, "EnvironmentConfig");
        
            inputDataset = addEnvironmentContext(inputDataset, benchmarkAOCS, time_s);
        
            simIn = Simulink.SimulationInput(testCase.HarnessName);
            simIn = simIn.setExternalInput(inputDataset);
            simIn = simIn.setModelParameter( ...
                "StartTime", "0", ...
                "StopTime", "1", ...
                "SolverType", "Fixed-step", ...
                "Solver", "FixedStepDiscrete", ...
                "FixedStep", "1", ...
                "SaveOutput", "on", ...
                "OutputSaveName", "yout", ...
                "SaveFormat", "Dataset");
        
            simOut = sim(simIn);
            yout = simOut.get("yout");

            rho_kg_m3 = squeeze(yout{1}.Values.Data);
            T_local_K = squeeze(yout{4}.Values.Data);
            T_exo_K = squeeze(yout{5}.Values.Data);
            
            rho_kg_m3 = rho_kg_m3(1);
            T_local_K = T_local_K(1);
            T_exo_K = T_exo_K(1);

            expectedRho_kg_m3 = 9.459569087245934e-12;
            expectedTlocal_K  = 843.2958374023438;
            expectedTexo_K    = 844.1524658203125;
            
            % Frozen reference generated with the official CNES DTM2020 F107/Kp
            % Fortran implementation for:
            % altitude=300 km, latitude=0, longitude=0,
            % day_of_year=180.5, local_solar_time=pi,
            % F10.7=80, F10.7_81d=80, Kp=3.
            testCase.verifyEqual( ...
                rho_kg_m3, expectedRho_kg_m3, ...
                "RelTol", 1e-5);
            
            testCase.verifyEqual( ...
                T_local_K, expectedTlocal_K, ...
                "AbsTol", 2e-3);
            
            testCase.verifyEqual( ...
                T_exo_K, expectedTexo_K, ...
                "AbsTol", 2e-3);

        end
    end
end

function names = rootPortNames(modelName, blockType)
    blocks = find_system(modelName, ...
        "SearchDepth", 1, "BlockType", blockType);
    ports = cellfun(@(block) ...
        str2double(get_param(block, "Port")), blocks);
    [~, order] = sort(ports);
    names = string(cellfun(@(block) ...
        get_param(block, "Name"), blocks, ...
        "UniformOutput", false));
    names = names(order);
end

function dataset = addInput(dataset, name, values, time_s)
    signal = timeseries(values, time_s);
    signal.Name = name;
    dataset = dataset.addElement(signal, name);
end

function bus = environmentConfigTimeseries(config, time_s)
    bus = struct();
    names = fieldnames(config);
    
    for k = 1:numel(names)
        name = names{k};
        bus.(name) = constantTimeseries(config.(name), time_s);
    end
end

function signal = constantTimeseries(value, time_s)
    sampleCount = numel(time_s);
    
    if isscalar(value)
        data = repmat(value, sampleCount, 1);
    elseif isrow(value)
        data = repmat(value, sampleCount, 1);
    else
        data = repmat(value, 1, 1, sampleCount);
    end
    
    signal = timeseries(data, time_s);
end

function dataset = addEnvironmentContext(dataset, AOCS, time_s)
    sampleCount = numel(time_s);
    epochUtc = AOCS.Epoch.Utc;
    earthOrientation = AOCS.Environment.EarthOrientation;

    epochDateTime = datetime( ...
        epochUtc(1), epochUtc(2), epochUtc(3), ...
        epochUtc(4), epochUtc(5), epochUtc(6), ...
        "TimeZone", "UTC");

    sampleDateTime = epochDateTime + seconds(time_s);
    utc = datevec(sampleDateTime);

    decimalYear = decyear( ...
        utc(:, 1), utc(:, 2), utc(:, 3), ...
        utc(:, 4), utc(:, 5), utc(:, 6));

    dataset = addInput(dataset, ...
        "EnvironmentContext_decimal_year", ...
        decimalYear, time_s);
    
    dataset = addInput(dataset, ...
        "EnvironmentContext_mu_m3_s2", ...
        repmat(AOCS.Orbit.CentralBodyConstants.mu_m3_s2, ...
            sampleCount, 1), ...
        time_s);

    dataset = addInput(dataset, ...
        "EnvironmentContext_epoch_utc", ...
        repmat(reshape(epochUtc, 6, 1), ...
            1, 1, sampleCount), ...
        time_s);
    
    dataset = addInput(dataset, ...
        "EnvironmentContext_t_s", ...
        time_s, time_s);
    
    dataset = addInput(dataset, ...
        "EnvironmentContext_epoch_tdb_jd", ...
        repmat(AOCS.Epoch.TdbJulianDate, sampleCount, 1), ...
        time_s);
    
    dataset = addInput(dataset, ...
        "EnvironmentContext_delta_at_s", ...
        repmat(earthOrientation.DeltaAT_s, sampleCount, 1), ...
        time_s);
    
    dataset = addInput(dataset, ...
        "EnvironmentContext_delta_ut1_s", ...
        repmat(earthOrientation.DeltaUT1_s, sampleCount, 1), ...
        time_s);
    
    dataset = addInput(dataset, ...
        "EnvironmentContext_polar_motion_rad", ...
        repmat(earthOrientation.PolarMotion_rad, sampleCount, 1), ...
        time_s);
    
    dataset = addInput(dataset, ...
        "EnvironmentContext_d_cip_rad", ...
        repmat(earthOrientation.DCIP_rad, sampleCount, 1), ...
        time_s);
end
