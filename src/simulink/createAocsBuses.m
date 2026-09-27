function createAocsBuses()
%CREATEAOCSBUSES Install the shared interface schema without loading a scenario.
createConfigBus("base");
createOrbitConfigBus("base");
createEnvironmentConfigBus("base");
createSgp4ConfigBus("base");
createAttitudeInitializationConfigBus("base");
createGNCConfigBus("base");
createAttitudeStateBus("base");
createOrbitStateBus("base");
createReferenceVectorBus("base");
createAttitudeInitializationBus("base");
createEnvironmentContextBus("base");
createAtmosphereBus("base");
createMagneticFieldBus("base");
createSunBus("base");
createIlluminationBus("base");
createSrpBus("base");
createDisturbanceBus("base");
createEnvironmentBus("base");
createPlantStateBus("base");
createGyroConfigBus("base");
createMagnetometerConfigBus("base");
createCoarseSunSensorConfigBus("base");
createGnssConfigBus("base");
createSensorConfigBus("base");
createGyroMeasurementBus("base");
createMagnetometerMeasurementBus("base");
createCoarseSunSensorMeasurementBus("base");
createGnssMeasurementBus("base");
createSensorMeasurementBus("base");
createSensorReportBus("base");

end
