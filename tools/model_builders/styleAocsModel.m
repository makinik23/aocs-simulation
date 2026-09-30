function styleAocsModel(model)
% Description:
%   Applies colors, fonts, and content-preview settings to the AOCS model.
%   Block positions and signal routes remain entirely user-controlled.
%
% Arguments:
%   model - Loaded Simulink model name.
%
% Outputs:
%   None. The caller decides whether to save the modified model.

model = string(model);

architectureBlocks = ["Flight Dynamics", "Sensors", "Drivers", "GNC"];
architectureColors = [
    0.752941 0.360784 0.984314
    0.600000 0.800000 1.000000
    1.000000 0.780000 0.420000
    0.560000 0.860000 0.660000
];

sensorBlocks = ["Gyro", "Magnetometer", "Coarse Sun Sensors", "GNSS"];
driverBlocks = sensorBlocks + " Driver";

styleArchitecture(model, architectureBlocks, architectureColors);
styleImplementationLayer(model + "/Sensors", sensorBlocks);
styleImplementationLayer(model + "/Drivers", driverBlocks);
end

function styleArchitecture(model, blockNames, blockColors)
for index = 1:numel(blockNames)
    block = model + "/" + blockNames(index);
    set_param(block, ...
        "BackgroundColor", formatRgbColor(blockColors(index, :)), ...
        "ForegroundColor", "black", ...
        "FontSize", 12, ...
        "ContentPreviewEnabled", "on");
end
end

function styleImplementationLayer(parent, blockNames)
for blockName = blockNames
    block = parent + "/" + blockName;
    if getSimulinkBlockHandle(block) <= 0
        continue;
    end

    set_param(block, ...
        "BackgroundColor", "white", ...
        "ForegroundColor", "black", ...
        "FontSize", 11, ...
        "ContentPreviewEnabled", "off");
end
end

function color = formatRgbColor(rgb)
color = sprintf("[%.6f, %.6f, %.6f]", rgb);
end
