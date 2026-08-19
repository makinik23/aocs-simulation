classdef SentmanPanelAerodynamicsTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addEnvironmentPath(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            addpath(fullfile(projectRoot, "src", "environment"));
            addpath(fullfile(projectRoot, "src", "config"));
        end
    end

    methods (Test)
        function inertialForceUsesBodyToInertialTransform(testCase)
            angle = pi / 2.0;
            C_BI = [cos(angle), sin(angle), 0.0; ...
                -sin(angle), cos(angle), 0.0; 0.0, 0.0, 1.0];
            [normals, areas, centers] = panelGeometry([0.1; 0.1; 0.3], zeros(3, 1));
            [forceBody, ~, forceInertial] = evaluateModel( ...
                [-7600.0; 0.0; 0.0], C_BI, normals, areas, centers, 1.0);

            testCase.verifyEqual(forceInertial, C_BI.' * forceBody, "AbsTol", 1e-20);
        end

        function disabledModelReturnsZeros(testCase)
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
