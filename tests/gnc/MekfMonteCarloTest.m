classdef MekfMonteCarloTest < matlab.unittest.TestCase
    % Description:
    %   Runs seeded noise and fault ensembles through the saved MEKF subsystem.

    methods (Test)
        function oneSeedReplaysAndSavesReport(testCase)
            % Description:
            %   Confirms exact seed replay and a self-contained MAT report.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            file=string(tempname)+'.mat';
            cleanup=onCleanup(@() delete(file));
            first=runMekfMonteCarlo(Seeds=17,Duration_s=12, ...
                Scenarios="nominal",SaveFile=file);
            replay=runMekfMonteCarlo(Seeds=17,Duration_s=12,Scenarios="nominal");
            saved=load(file,'report');
            testCase.verifyEqual(first.Trials,replay.Trials);
            testCase.verifyEqual(saved.report.Trials,first.Trials);
            testCase.verifyEqual(first.Seeds,17);
            testCase.verifyTrue(isfield(first,'TruthModel'));
            testCase.verifyTrue(isfield(first,'Tuning'));
        end

        function seededEnsembleConvergesAndGuardsFaults(testCase)
            % Description:
            %   Checks ensemble attitude, bias, covariance and health behavior.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            report=runMekfMonteCarlo(Seeds=1:4,Duration_s=60, ...
                Scenarios=["matched","nominal","outages"]);
            trials=report.Trials;
            testCase.verifyEqual(numel(trials),12);
            testCase.verifyTrue(all([trials.Initialized]));
            testCase.verifyTrue(all([trials.CovarianceFinite]));
            testCase.verifyGreaterThan(min([trials.MinimumCovarianceEigenvalue]),-1e-12);
            testCase.verifyLessThan(max([trials.AttitudeP95_deg]),2);
            testCase.verifyLessThan(max([trials.FinalBiasError_rad_s]),1e-3);
            testCase.verifyEqual(report.Summary.FalseSafeSamples,0);
            testCase.verifyEqual(report.Summary.FalseSafeSamples2deg,0);
            testCase.verifyLessThan(report.Summary.MaximumUsableAttitudeError_deg,2);
            testCase.verifyTrue(all(isfinite([trials.NEESMean])));
            testCase.verifyTrue(all(isfinite([trials.MagNISMean])));
            testCase.verifyTrue(all(isfinite([trials.SunNISMean])));
            matched=trials([trials.Scenario]=="matched");
            testCase.verifyGreaterThan(mean([matched.NEESMean]),2);
            testCase.verifyLessThan(mean([matched.NEESMean]),15);
            testCase.verifyLessThan(max([matched.AttitudeP95_deg]),1);
            stressed=trials([trials.Scenario]=="outages");
            testCase.verifyTrue(all([stressed.GyroOutageGuarded]));
            testCase.verifyTrue(all([stressed.VectorOutageGuarded]));
            testCase.verifyTrue(all([stressed.OutlierRejected]));
        end
    end

    methods (Test, TestTags=["Integration","FullPlant"])
        function fullPlantSensorSeedEnsemble(testCase)
            % Description:
            %   Checks estimator behavior with seeded physical sensors and references.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.
            report=runMekfMonteCarlo(Scope="plant",Seeds=1:2,Duration_s=50);
            trials=report.Trials;
            testCase.verifyTrue(all([trials.Initialized]));
            testCase.verifyTrue(all([trials.FinalControlUsable]));
            testCase.verifyLessThan(max([trials.AttitudeP95_deg]),1.5);
            testCase.verifyLessThan(max([trials.FinalBiasError_rad_s]),5e-4);
            testCase.verifyEqual(sum([trials.FalseSafeSamples]),0);
        end
    end
end
