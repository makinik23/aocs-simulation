classdef Dtm2020NativeTest < matlab.unittest.TestCase
    properties (SetAccess = private)
        ProjectRoot
        NativeArtifacts
    end

    methods (TestClassSetup)
        function buildNativeBackend(testCase)
            testCase.ProjectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            addpath(fullfile(testCase.ProjectRoot, "tools"));
            testCase.NativeArtifacts = buildDtm2020Native();
        end
    end

    methods (Test)
        function operationalBenchmarkMatchesCnesReference(testCase)
            addpath(testCase.NativeArtifacts.BuildDirectory);
            clear dtm2020_mex;
            [rho, ~, localTemperature, exosphericTemperature] = dtm2020_mex( ...
                char(testCase.NativeArtifacts.CoefficientFile), ...
                180.0, [80.0; 0.0], [80.0; 0.0], ...
                [3.0; 0.0; 3.0; 0.0], 300.0, 3.1415, 0.0, 0.0);

            testCase.verifyEqual(rho, 0.94719e-14, "RelTol", 1e-5);
            testCase.verifyEqual(localTemperature, 843.243, "AbsTol", 2e-3);
            testCase.verifyEqual(exosphericTemperature, 844.099, "AbsTol", 2e-3);
        end

        function convertsAllSpeciesAndUncertainty(testCase)
            addpath(testCase.NativeArtifacts.BuildDirectory);
            clear dtm2020_mex;
            [rho, uncertainty, localTemperature, exosphericTemperature, species] = ...
                dtm2020_mex(char(testCase.NativeArtifacts.CoefficientFile), ...
                180.0, [180.0; 0.0], [180.0; 0.0], ...
                [3.0; 0.0; 3.0; 0.0], 300.0, pi, 0.0, 0.0);

            testCase.verifyGreaterThan(rho, 0.0);
            testCase.verifyGreaterThan(uncertainty, 0.0);
            testCase.verifyGreaterThan(localTemperature, 0.0);
            testCase.verifyGreaterThanOrEqual(exosphericTemperature, localTemperature);
            testCase.verifySize(species, [6 1]);
            testCase.verifyGreaterThanOrEqual(species, zeros(6, 1));
        end

        function nativeOutputConvertsToSiContract(testCase)
            nativeOutput = [1.0e-15; 25.0; 900.0; 1000.0; ...
                1.0e-18; 2.0e-18; 3.0e-16; 4.0e-16; 2.0e-16; 1.0e-18];
            config = struct( ...
                "atmosphere_enabled", 1.0, ...
                "atmosphere_uncertainty_enabled", 1.0, ...
                "rho_scale_factor", 2.0);

            [rho, rhoRaw, sigma, localTemperature, exosphericTemperature, ...
                nO, nN2, nO2, nHe, nH, nN, atmosphereVelocity] = ...
                dtm2020.postprocessNativeOutput(nativeOutput, ...
                [7000e3; 0.0; 0.0], config);

            testCase.verifyEqual(rhoRaw, 1.0e-12, "RelTol", 1e-14);
            testCase.verifyEqual(rho, 2.0e-12, "RelTol", 1e-14);
            testCase.verifyEqual(sigma, 0.5e-12, "RelTol", 1e-14);
            testCase.verifyEqual(localTemperature, 900.0);
            testCase.verifyEqual(exosphericTemperature, 1000.0);
            testCase.verifyGreaterThan([nO; nN2; nO2; nHe; nH; nN], zeros(6, 1));
            testCase.verifyEqual(atmosphereVelocity, [0.0; 510.44805; 0.0], ...
                "AbsTol", 1e-8);
        end
    end
end
