classdef GnssErrorModelTest < matlab.unittest.TestCase

    properties
        ProjectRoot
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            % Description:
            %   Locates the project root and adds the GNSS model source path.
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
        function alignedOrbitMapsRtnComponentsDirectly(testCase)
            % Description:
            %   Verifies RTN-to-inertial mapping for an orbit aligned with
            %   the inertial coordinate axes.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            rTruth_I_m = [7.0e6; 0.0; 0.0];
            vTruth_I_m_s = [0.0; 7.5e3; 0.0];
            positionError_RTN_m = [10.0; 20.0; 30.0];
            velocityError_RTN_m_s = [0.01; 0.02; 0.03];

            [rMeasured, vMeasured, C_I_RTN] = applyGnssRtnErrors( ...
                rTruth_I_m, vTruth_I_m_s, positionError_RTN_m, ...
                velocityError_RTN_m_s);

            testCase.verifyEqual(C_I_RTN, eye(3), "AbsTol", 1.0e-15);
            testCase.verifyEqual(rMeasured, ...
                rTruth_I_m + positionError_RTN_m, "AbsTol", 1.0e-12);
            testCase.verifyEqual(vMeasured, ...
                vTruth_I_m_s + velocityError_RTN_m_s, "AbsTol", 1.0e-12);
        end

        function rotatedOrbitUsesRightHandedRtnFrame(testCase)
            % Description:
            %   Verifies radial, transverse, and normal axes for a rotated
            %   prograde equatorial orbit.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            rTruth_I_m = [0.0; 7.0e6; 0.0];
            vTruth_I_m_s = [-7.5e3; 0.0; 0.0];
            expectedC = [0.0 -1.0 0.0; 1.0 0.0 0.0; 0.0 0.0 1.0];

            [rMeasured, ~, C_I_RTN] = applyGnssRtnErrors( ...
                rTruth_I_m, vTruth_I_m_s, [1.0; 2.0; 3.0], zeros(3, 1));

            testCase.verifyEqual(C_I_RTN, expectedC, "AbsTol", 1.0e-15);
            testCase.verifyEqual(rMeasured - rTruth_I_m, ...
                expectedC * [1.0; 2.0; 3.0], "AbsTol", 1.0e-12);
            testCase.verifyEqual(C_I_RTN' * C_I_RTN, eye(3), ...
                "AbsTol", 1.0e-15);
            testCase.verifyEqual(det(C_I_RTN), 1.0, "AbsTol", 1.0e-15);
        end

        function degenerateStateFallsBackToInertialAxes(testCase)
            % Description:
            %   Verifies deterministic finite behavior for an invalid zero
            %   truth state instead of producing NaN measurements.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            [rMeasured, vMeasured, C_I_RTN] = applyGnssRtnErrors( ...
                zeros(3, 1), zeros(3, 1), [1.0; 2.0; 3.0], ...
                [0.1; 0.2; 0.3]);

            testCase.verifyEqual(C_I_RTN, eye(3));
            testCase.verifyEqual(rMeasured, [1.0; 2.0; 3.0]);
            testCase.verifyEqual(vMeasured, [0.1; 0.2; 0.3]);
        end
    end
end
