classdef SentmanPanelAerodynamicsTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addEnvironmentPath(~)
            % Description:
            %   Adds source and test helper paths required by Sentman tests.
            %
            % Arguments:
            %   None.
            %
            % Outputs:
            %   None.

            setupAocsPaths(projectRoot(), true);
        end
    end

    methods (Test)
        function inertialForceUsesBodyToInertialTransform(testCase)
            % Description:
            %   Verifies that the Sentman model transforms body-frame force into
            %   inertial-frame force with the supplied attitude matrix.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            angle = pi / 2.0;
            C_BI = [cos(angle), sin(angle), 0.0; ...
                -sin(angle), cos(angle), 0.0; 0.0, 0.0, 1.0];
            [normals, areas, centers] = panelGeometry([0.1; 0.1; 0.3], zeros(3, 1));
            [forceBody, ~, forceInertial] = evaluateModel( ...
                [-7600.0; 0.0; 0.0], C_BI, normals, areas, centers, 1.0);

            testCase.verifyEqual(forceInertial, C_BI.' * forceBody, "AbsTol", 1e-20);
        end

        function disabledModelReturnsZeros(testCase)
            % Description:
            %   Verifies that disabling the Sentman model zeros force, moment,
            %   acceleration, per-panel force, and dynamic-pressure products.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            [normals, areas, centers] = panelGeometry([0.1; 0.1; 0.3], zeros(3, 1));
            [forceBody, momentBody, forceInertial, acceleration, panelForces, qDynamic] = ...
                evaluateModel([-7600.0; 0.0; 0.0], eye(3), normals, areas, centers, 0.0);

            testCase.verifyEqual([forceBody; momentBody; forceInertial; acceleration], zeros(12, 1));
            testCase.verifyEqual(panelForces, zeros(3, 6));
            testCase.verifyEqual(qDynamic, 0.0);
        end
    end
end

function [forceBody, momentBody, forceInertial, acceleration, panelForces, qDynamic] = ...
        evaluateModel(flowBody, C_BI, normals, areas, centers, enabled)
% Description:
%   Evaluates computeSentmanPanelAerodynamics with representative multispecies
%   atmospheric inputs.
%
% Arguments:
%   flowBody - 3-by-1 molecular flow velocity in body axes [m/s].
%   C_BI - 3-by-3 DCM mapping inertial vectors into body axes.
%   normals - 3-by-N panel normal matrix in body axes.
%   areas - N-by-1 panel area vector [m^2].
%   centers - 3-by-N panel center matrix in body axes [m].
%   enabled - Scalar enable flag.
%
% Outputs:
%   forceBody - 3-by-1 aerodynamic force in body axes [N].
%   momentBody - 3-by-1 aerodynamic moment in body axes [N*m].
%   forceInertial - 3-by-1 aerodynamic force in inertial axes [N].
%   acceleration - 3-by-1 aerodynamic acceleration in inertial axes [m/s^2].
%   panelForces - 3-by-N per-panel aerodynamic force matrix [N].
%   qDynamic - Dynamic pressure [N/m^2].

rho = 1.0e-12;
temperature = 900.0;
amu = 1.66053906660e-27;
massNumbers = [16.0; 28.0; 32.0; 4.0; 1.0; 14.0];
massFractions = [0.70; 0.12; 0.03; 0.10; 0.03; 0.02];
numberDensities = rho .* massFractions ./ (amu .* massNumbers);

[forceBody, momentBody, forceInertial, acceleration, panelForces, qDynamic] = ...
    computeSentmanPanelAerodynamics(flowBody, rho, temperature, ...
    numberDensities, C_BI, 3.71565, enabled, normals, areas, centers, ...
    300.0 * ones(6, 1), 0.9 * ones(6, 1));
end

function [normals, areas, centers] = panelGeometry(dimensions, centerOfMass)
% Description:
%   Builds six-face cuboid panel normals, areas, and centers about a center of mass.
%
% Arguments:
%   dimensions - 3-by-1 cuboid dimensions [m].
%   centerOfMass - 3-by-1 center-of-mass offset [m].
%
% Outputs:
%   normals - 3-by-6 face-normal matrix.
%   areas - 6-by-1 face-area vector [m^2].
%   centers - 3-by-6 face-center matrix [m].

dx = dimensions(1);
dy = dimensions(2);
dz = dimensions(3);
normals = [1.0, -1.0, 0.0, 0.0, 0.0, 0.0; ...
    0.0, 0.0, 1.0, -1.0, 0.0, 0.0; ...
    0.0, 0.0, 0.0, 0.0, 1.0, -1.0];
areas = [dy * dz; dy * dz; dx * dz; dx * dz; dx * dy; dx * dy];
centers = [dx / 2.0, -dx / 2.0, 0.0, 0.0, 0.0, 0.0; ...
    0.0, 0.0, dy / 2.0, -dy / 2.0, 0.0, 0.0; ...
    0.0, 0.0, 0.0, 0.0, dz / 2.0, -dz / 2.0] - centerOfMass;
end
