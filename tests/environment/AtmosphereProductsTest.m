classdef AtmosphereProductsTest < matlab.unittest.TestCase

    properties
        ProjectRoot
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
        end
    end

    methods (Test)
        function atmosphereBusDefinesRuntimeContract(testCase)
            % Description:
            %   Verifies that the atmosphere bus exposes the expected runtime
            %   product names and vector dimensions.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            bus = createAocsAtmosphereBus();
            names = string({bus.Elements.Name});

            expectedNames = [ ...
                "rho_kg_m3", ...
                "rho_raw_kg_m3", ...
                "rho_uncertainty_1sigma_kg_m3", ...
                "T_local_K", ...
                "T_exo_K", ...
                "n_O_m3", ...
                "n_N2_m3", ...
                "n_O2_m3", ...
                "n_He_m3", ...
                "n_H_m3", ...
                "n_N_m3", ...
                "v_atm_I_m_s"];

            testCase.verifyEqual(names, expectedNames);
            testCase.verifyEqual(bus.Elements(12).Dimensions, [3 1]);
        end

        function environmentBusPublishesAtmosphereProducts(testCase)
            % Description:
            %   Verifies that the top-level environment bus includes the
            %   atmosphere-products bus.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            createAocsAtmosphereBus("base");
            bus = createAocsEnvironmentBus();
            names = string({bus.Elements.Name});
            atmosphereIndex = find(names == "Atmosphere", 1);

            testCase.verifyNotEmpty(atmosphereIndex);
            testCase.verifyEqual(string(bus.Elements(atmosphereIndex).DataType), ...
                "Bus: AOCS_AtmosphereBus");
        end

        function enabledAtmosphereProducesFiniteScaledProducts(testCase)
            % Description:
            %   Checks that the enabled atmosphere model returns finite, scaled
            %   density, temperature, species, and velocity products.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            config = nominalEnvironmentConfig(2.5, true);
            r_I_m = [6871000.0; 0.0; 0.0];
            v_I_m_s = [0.0; 7600.0; 0.0];
            lla = [0.0; 0.0; 500e3];

            context = nominalEnvironmentContext();
            [rho, rhoRaw, rhoUncertainty, TLocal, TExo, nO, nN2, nO2, nHe, ...
                nH, nN, vAtm] = computeAtmosphereProducts( ...
                r_I_m, v_I_m_s, lla, config, context);

            testCase.verifyGreaterThan(rhoRaw, 0.0);
            testCase.verifyEqual(rho, config.rho_scale_factor * rhoRaw, ...
                "RelTol", 1e-14);
            testCase.verifyEqual(rhoUncertainty, 0.3 * rho, "RelTol", 1e-14);
            testCase.verifyGreaterThan(TLocal, 0.0);
            testCase.verifyGreaterThanOrEqual(TExo, TLocal);
            testCase.verifyGreaterThan(nO, 0.0);
            testCase.verifyGreaterThan(nN2, 0.0);
            testCase.verifyGreaterThan(nO2, 0.0);
            testCase.verifyGreaterThan(nHe, 0.0);
            testCase.verifyGreaterThan(nH, 0.0);
            testCase.verifyEqual(nN, 0.0);
            testCase.verifyTrue(all(isfinite([rho, rhoRaw, rhoUncertainty, ...
                TLocal, TExo, nO, nN2, nO2, nHe, nH, nN, vAtm(:).'])));
        end

        function atmosphereVelocityUsesEarthCorotation(testCase)
            % Description:
            %   Verifies that the atmosphere velocity product follows the
            %   configured Earth corotation model.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            config = nominalEnvironmentConfig(1.0, true);
            r_I_m = [6871000.0; 1000.0; -2000.0];
            v_I_m_s = [0.0; 7600.0; 0.0];
            lla = [12.0; 30.0; 450e3];

            [~, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~, vAtm] = ...
                computeAtmosphereProducts(r_I_m, v_I_m_s, lla, config, ...
                nominalEnvironmentContext());

            omegaEarth_I_rad_s = [0.0; 0.0; 7.2921150e-5];
            expected = cross(omegaEarth_I_rad_s, r_I_m);

            testCase.verifyEqual(vAtm, expected, "AbsTol", 1e-12);
        end

        function fallbackDensityDecreasesWithAltitude(testCase)
            % Description:
            %   Checks the fallback exponential-density model for monotonic
            %   decrease over representative LEO altitudes.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            config = nominalEnvironmentConfig(1.0, false);
            r_I_m = [6871000.0; 0.0; 0.0];
            v_I_m_s = [0.0; 7600.0; 0.0];

            context = nominalEnvironmentContext();
            [rhoLow] = computeAtmosphereProducts(r_I_m, v_I_m_s, ...
                [0.0; 0.0; 400e3], config, context);
            [rhoHigh] = computeAtmosphereProducts(r_I_m, v_I_m_s, ...
                [0.0; 0.0; 800e3], config, context);

            testCase.verifyGreaterThan(rhoLow, rhoHigh);
        end

        function disabledAtmosphereZerosProducts(testCase)
            % Description:
            %   Verifies that disabling the atmosphere model zeros all published
            %   atmosphere products.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            config = nominalEnvironmentConfig(1.0, true);
            config.atmosphere_enabled = 0.0;

            [rho, rhoRaw, rhoUncertainty, TLocal, TExo, nO, nN2, nO2, nHe, ...
                nH, nN, vAtm] = ...
                computeAtmosphereProducts([6871000.0; 0.0; 0.0], [0.0; 7600.0; 0.0], ...
                [0.0; 0.0; 500e3], config, nominalEnvironmentContext());

            testCase.verifyEqual([rho, rhoRaw, rhoUncertainty, TLocal, TExo, ...
                nO, nN2, nO2, nHe, nH, nN], zeros(1, 11));
            testCase.verifyEqual(vAtm, zeros(3, 1));
        end
    end
end

function config = nominalEnvironmentConfig(rhoScaleFactor, uncertaintyEnabled)
% Description:
%   Builds a representative environment-config struct for atmosphere tests.
%
% Arguments:
%   rhoScaleFactor - Density scale factor applied by computeAtmosphereProducts.
%   uncertaintyEnabled - Logical flag enabling density uncertainty output.
%
% Outputs:
%   config - Struct matching the atmosphere configuration bus fields.

config = struct();
config.atmosphere_enabled = 1.0;
config.atmosphere_model_id = 1.0;
config.atmosphere_mode_id = 1.0;
config.atmosphere_space_weather_source_id = 1.0;
config.atmosphere_uncertainty_enabled = double(uncertaintyEnabled);
config.rho_scale_factor = rhoScaleFactor;
config.f10_7_sfu = 150.0;
config.f10_7_81d_sfu = 150.0;
config.kp = 2.0;
config.f30_sfu = 150.0;
config.f30_81d_sfu = 150.0;
config.hp60 = 2.0;
end

function context = nominalEnvironmentContext()
% Description:
%   Builds a representative environment-context struct for atmosphere tests.
%
% Arguments:
%   None.
%
% Outputs:
%   context - Struct matching the environment context bus fields.

context = struct();
context.epoch_utc = [2026.0; 1.0; 1.0; 0.0; 0.0; 0.0];
context.t_s = 0.0;
end
