function referenceFile = generateThreeUCuboidSentmanReference(adbsatRoot)
%GENERATETHREEUCUBOIDSENTMANREFERENCE Build a full-mesh ADBSat fixture.
%   ADBSat imports the 3U OBJ mesh, evaluates its Sentman panel model, and
%   returns force and moment coefficients over an AoA/AoS grid.

arguments
    adbsatRoot (1, 1) string
end

requiredFiles = [ ...
    fullfile(adbsatRoot, "ADBSatImport.m"), ...
    fullfile(adbsatRoot, "ADBSatFcn.m"), ...
    fullfile(adbsatRoot, "toolbox", "calc", "calc_coeff.m")];
if any(~isfile(requiredFiles))
    error("AOCS:Validation:MissingADBSat", ...
        "A complete ADBSat checkout was not found under %s.", adbsatRoot);
end

originalPath = path;
pathCleanup = onCleanup(@() path(originalPath));
addpath(genpath(adbsatRoot));

projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
geometryFile = fullfile(projectRoot, "validation", "aerodynamics", ...
    "geometry", "cubesat_3u.obj");
referenceFile = fullfile(projectRoot, "validation", "aerodynamics", ...
    "data", "sentman_3u_cuboid_reference.csv");

workingDirectory = string(tempname);
mkdir(workingDirectory);
directoryCleanup = onCleanup(@() removeWorkingDirectory(workingDirectory));
resultsDirectory = fullfile(workingDirectory, "results");
mkdir(resultsDirectory);

meshFile = ADBSatImport(char(geometryFile), char(workingDirectory), 0);

atomicMassUnitKg = 1.66053906660e-27;
boltzmannJK = 1.380649e-23;
oxygenMassKg = 16.0 * atomicMassUnitKg;
speedMps = 7600.0;
rhoKgM3 = 1.0e-12;
atmosphereTemperatureK = 900.0;
wallTemperatureK = 300.0;
alpha = 0.9;
spacecraftMassKg = 3.71565;

parameters = struct();
parameters.gsi_model = 'sentman';
parameters.alpha = alpha;
parameters.Tw = wallTemperatureK;
parameters.Tinf = atmosphereTemperatureK;
parameters.s = speedMps * sqrt(oxygenMassKg / ...
    (2.0 * boltzmannJK * atmosphereTemperatureK));

anglesDeg = -85:10:85;
outputFile = ADBSatFcn(char(meshFile), char(resultsDirectory), parameters, ...
    anglesDeg, anglesDeg, 0, 0, 0, 1, 0);
output = load(outputFile, "aedb");
database = output.aedb;

aoaRad = database.aoa(:);
aosRad = database.aos(:);
sampleCount = numel(aoaRad);
qDynamicNm2 = 0.5 * rhoKgM3 * speedMps^2;
areaReferenceM2 = database.AreaRef;
lengthReferenceM = database.LenRef;

vFlowBodyMps = zeros(sampleCount, 3);
forceBodyN = zeros(sampleCount, 3);
momentBodyNm = zeros(sampleCount, 3);
bodyToGeometric = diag([1.0, -1.0, -1.0]);

for sampleIndex = 1:sampleCount
    aoa = aoaRad(sampleIndex);
    aos = aosRad(sampleIndex);
    bodyToWind = [ ...
        cos(aos) * cos(aoa), sin(aos), sin(aoa) * cos(aos); ...
        -sin(aos) * cos(aoa), cos(aos), -sin(aoa) * sin(aos); ...
        -sin(aoa), 0.0, cos(aoa)];
    windToGeometric = bodyToGeometric * bodyToWind.';

    flowDirectionGeometric = windToGeometric * [-1.0; 0.0; 0.0];
    forceCoefficientWind = [ ...
        database.aero.Cf_wX(sampleIndex); ...
        database.aero.Cf_wY(sampleIndex); ...
        database.aero.Cf_wZ(sampleIndex)];
    momentCoefficientBody = [ ...
        database.aero.Cm_BX(sampleIndex); ...
        database.aero.Cm_BY(sampleIndex); ...
        database.aero.Cm_BZ(sampleIndex)];

    forceCoefficientGeometric = windToGeometric * forceCoefficientWind;
    momentCoefficientGeometric = bodyToGeometric * momentCoefficientBody;

    vFlowBodyMps(sampleIndex, :) = ...
        (speedMps * flowDirectionGeometric).';
    forceBodyN(sampleIndex, :) = ...
        (qDynamicNm2 * areaReferenceM2 * forceCoefficientGeometric).';
    momentBodyNm(sampleIndex, :) = ...
        (qDynamicNm2 * areaReferenceM2 * lengthReferenceM * ...
        momentCoefficientGeometric).';
end

reference = table();
reference.case_id = compose("aoa_%+03d_aos_%+03d", ...
    round(rad2deg(aoaRad)), round(rad2deg(aosRad)));
reference.aoa_deg = rad2deg(aoaRad);
reference.aos_deg = rad2deg(aosRad);
reference.v_flow_x_m_s = vFlowBodyMps(:, 1);
reference.v_flow_y_m_s = vFlowBodyMps(:, 2);
reference.v_flow_z_m_s = vFlowBodyMps(:, 3);
reference.rho_kg_m3 = rhoKgM3 * ones(sampleCount, 1);
reference.T_atm_K = atmosphereTemperatureK * ones(sampleCount, 1);
reference.T_wall_K = wallTemperatureK * ones(sampleCount, 1);
reference.alpha_E = alpha * ones(sampleCount, 1);
reference.mass_kg = spacecraftMassKg * ones(sampleCount, 1);
reference.force_x_reference_N = forceBodyN(:, 1);
reference.force_y_reference_N = forceBodyN(:, 2);
reference.force_z_reference_N = forceBodyN(:, 3);
reference.moment_x_reference_Nm = momentBodyNm(:, 1);
reference.moment_y_reference_Nm = momentBodyNm(:, 2);
reference.moment_z_reference_Nm = momentBodyNm(:, 3);
reference.q_dynamic_reference_N_m2 = qDynamicNm2 * ones(sampleCount, 1);
reference.adbsat_area_reference_m2 = areaReferenceM2 * ones(sampleCount, 1);
reference.adbsat_length_reference_m = lengthReferenceM * ones(sampleCount, 1);

writetable(reference, referenceFile);
fprintf("Wrote %d full-mesh ADBSat 3U cases to %s.\n", ...
    height(reference), referenceFile);
end

function removeWorkingDirectory(directory)
if isfolder(directory)
    rmdir(directory, "s");
end
end
