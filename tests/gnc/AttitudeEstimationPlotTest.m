classdef AttitudeEstimationPlotTest < matlab.unittest.TestCase
    % Description:
    %   Checks end-to-end gyro truth logging and estimator comparison plots.

    methods (Test, TestTags = ["Integration", "FullPlant"])
        function plotsAlignedEstimateAndTruth(testCase)
            % Description:
            %   Runs a short plant simulation, then checks diagnostic products.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            setupAocsPaths([], true);
            [input, AOCS] = createAocsSimulationInput();
            modelCleanup = onCleanup(@() close_system(AOCS.Model.Name, 0));
            input = input.setModelParameter("StopTime", "12");
            out = sim(input);
            testCase.verifyClass(out.get("GyroTrueBias"), "timeseries");

            resultsFile = string(tempname) + ".mat";
            outputDirectory = string(tempname);
            mkdir(outputDirectory);
            fileCleanup = onCleanup(@() delete(resultsFile));
            imageCleanup = onCleanup(@() rmdir(outputDirectory, "s"));
            save(resultsFile, "out");

            originalVisibility = get(groot, "DefaultFigureVisible");
            set(groot, "DefaultFigureVisible", "off");
            visibilityCleanup = onCleanup(@() set(groot, "DefaultFigureVisible", originalVisibility));
            [figures, diagnostics] = plot_attitude_estimation_results(resultsFile, outputDirectory);
            figureCleanup = onCleanup(@() close(figures));

            testCase.verifyEqual(numel(figures), 2);
            testCase.verifyTrue(diagnostics.HasGyroBiasTruth);
            testCase.verifyEqual(numel(diagnostics.Time_s), numel(diagnostics.AttitudeError_deg));
            testCase.verifyTrue(all(isfinite(diagnostics.AttitudeError_deg(diagnostics.Initialized))));
            testCase.verifyTrue(all(isfinite(diagnostics.GyroBiasError_B_rad_s(diagnostics.Initialized, :)), "all"));
            testCase.verifyTrue(isfile(fullfile(outputDirectory, "attitude_estimate.png")));
            testCase.verifyTrue(isfile(fullfile(outputDirectory, "body_rate_and_gyro_bias.png")));
        end
    end
end
