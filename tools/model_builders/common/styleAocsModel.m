function styleAocsModel(model)
% Description:
%   Applies the deliberate top-level, Sensors, and Drivers layout to the
%   AOCS model.
%
% Arguments:
%   model - Loaded Simulink model name.
%
% Outputs:
%   None. The caller decides whether to save the modified model.

model = string(model);
sensorSpecs = sensorDriverSpecs();

styleArchitecture(model);
styleDrivers(model, sensorSpecs);
styleSensors(model, sensorSpecs);

routeSystem(model);
routeSystem(model + "/Drivers");
routeSystem(model + "/Sensors");
routeSensorPlantStateInputs(model + "/Sensors");
end

function styleArchitecture(model)
blockNames = ["Flight Dynamics", "Sensors", "Drivers", "GNC"];
blockColors = [
    0.752941 0.360784 0.984314
    0.600000 0.800000 1.000000
    1.000000 0.780000 0.420000
    0.560000 0.860000 0.660000
];

leftMargin = 100;
topMargin = 90;
blockWidth = 400;
blockHeight = 500;
horizontalPitch = 420;

for index = 1:numel(blockNames)
    left = leftMargin + horizontalPitch*(index - 1);
    position = [left, topMargin, left + blockWidth, topMargin + blockHeight];
    set_param(model + "/" + blockNames(index), ...
        "Position", position, ...
        "BackgroundColor", formatRgbColor(blockColors(index, :)), ...
        "ForegroundColor", "black", ...
        "FontSize", 12, ...
        "ContentPreviewEnabled", "on");
end

unusedOutput = model + "/Unused Attitude Initialization";
if getSimulinkBlockHandle(unusedOutput) > 0
    outputCenterY = topMargin + blockHeight/2;
    outputLeft = leftMargin + horizontalPitch*numel(blockNames) - 40;
    set_param(unusedOutput, "Position", ...
        [outputLeft, outputCenterY - 10, outputLeft + 20, outputCenterY + 10]);
end
end

function styleDrivers(model, sensorSpecs)
drivers = model + "/Drivers";
set_param(drivers + "/SensorMeasurements", ...
    "Position", [30 330 60 350]);
set_param(drivers + "/Select Measurements", ...
    "Position", [120 0 125 680]);

firstDriverTop = 60;
driverPitch = 170;
for index = 1:numel(sensorSpecs)
    driverTop = firstDriverTop + driverPitch*(index - 1);
    driver = drivers + "/" + sensorSpecs(index).Name + " Driver";
    set_param(driver, ...
        "Position", [350 driverTop 550 driverTop + 100], ...
        "BackgroundColor", "white", ...
        "ForegroundColor", "black", ...
        "FontSize", 11, ...
        "ContentPreviewEnabled", "off");

    dataReady = drivers + "/" + sensorSpecs(index).Key + " DataReady";
    set_param(dataReady, ...
        "Position", [230 driverTop + 65 260 driverTop + 85]);
    set_param(driver + "/Receive Sample", "ContentPreviewEnabled", "off");
end

set_param(drivers + "/Sensor Report Bus Assembly", ...
    "Position", [685 25 690 705]);
set_param(drivers + "/SensorReports", ...
    "Position", [810 355 840 375]);
end

function styleSensors(model, sensorSpecs)
sensors = model + "/Sensors";
for index = 1:numel(sensorSpecs)
    sensor = sensors + "/" + sensorSpecs(index).Name;
    set_param(sensor, ...
        "BackgroundColor", "white", ...
        "ForegroundColor", "black", ...
        "FontSize", 11, ...
        "ContentPreviewEnabled", "off");

    sensorPorts = get_param(sensor, "PortHandles");
    dataReadyPortPosition = get_param(sensorPorts.Outport(2), "Position");
    dataReady = sensors + "/" + sensorSpecs(index).Key + " DataReady";
    set_param(dataReady, "Position", ...
        [420 dataReadyPortPosition(2) - 7 450 dataReadyPortPosition(2) + 7]);
end

set_param(sensors + "/Sensor Measurement Bus Assembly", ...
    "Position", [650 -19 655 661]);
set_param(sensors + "/SensorMeasurements", ...
    "Position", [810 311 840 331]);
end

function routeSensorPlantStateInputs(sensors)
selector = findPlantStateSelector(sensors);
selectorPorts = get_param(selector, "PortHandles");

routeEnvironmentBranches(selectorPorts.Outport(2));
routeOrbitStateSignal(selectorPorts.Outport(3));
end

function selector = findPlantStateSelector(sensors)
expectedOutputs = "AttitudeState,Environment,OrbitState";
selectors = string(find_system(sensors, ...
    "SearchDepth", 1, "BlockType", "BusSelector"));

for index = 1:numel(selectors)
    outputs = string(get_param(selectors(index), "OutputSignals"));
    if outputs == expectedOutputs
        selector = selectors(index);
        return;
    end
end

error("AOCS:ModelLayout:MissingPlantStateSelector", ...
    "Sensors must contain a Bus Selector with outputs %s.", expectedOutputs);
end

function routeEnvironmentBranches(sourcePort)
lineHandle = get_param(sourcePort, "Line");
sourcePoint = get_param(sourcePort, "Position");
forkPoint = [60 sourcePoint(2)];
set_param(lineHandle, "Points", [sourcePoint; forkPoint]);

branchLines = get_param(lineHandle, "LineChildren");
for branchLine = branchLines(:).'
    destinationPort = get_param(branchLine, "DstPortHandle");
    destinationPoint = get_param(destinationPort, "Position");
    routePoints = [
        forkPoint
        forkPoint(1) destinationPoint(2)
        destinationPoint
    ];
    set_param(branchLine, "Points", unique(routePoints, "rows", "stable"));
end
end

function routeOrbitStateSignal(sourcePort)
lineHandle = get_param(sourcePort, "Line");
destinationPort = get_param(lineHandle, "DstPortHandle");
sourcePoint = get_param(sourcePort, "Position");
destinationPoint = get_param(destinationPort, "Position");
routePoints = [
    sourcePoint
    -70 sourcePoint(2)
    -70 destinationPoint(2)
    destinationPoint
];
set_param(lineHandle, "Points", routePoints);
end

function routeSystem(system)
lineHandles = find_system(system, ...
    "FindAll", "on", "SearchDepth", 1, "Type", "line");

for lineHandle = lineHandles(:).'
    if get_param(lineHandle, "LineParent") > 0
        continue;
    end

    sourcePort = get_param(lineHandle, "SrcPortHandle");
    destinationPort = get_param(lineHandle, "DstPortHandle");
    if sourcePort <= 0 || ~isscalar(destinationPort) || destinationPort <= 0
        continue;
    end

    sourcePoint = get_param(sourcePort, "Position");
    destinationPoint = get_param(destinationPort, "Position");
    bendX = sourcePoint(1) + 0.4*(destinationPoint(1) - sourcePoint(1));
    routePoints = [
        sourcePoint
        bendX sourcePoint(2)
        bendX destinationPoint(2)
        destinationPoint
    ];
    set_param(lineHandle, "Points", unique(routePoints, "rows", "stable"));
end
end

function color = formatRgbColor(rgb)
color = sprintf("[%.6f, %.6f, %.6f]", rgb);
end
