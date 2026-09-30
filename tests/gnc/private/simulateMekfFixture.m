function s = simulateMekfFixture(f)
% Description:
%   Simulates the saved MEKF through public typed interfaces, without truth
%   wiring into the estimator. Missing vector reports disable corrections.
% Arguments:
%   f - Fixture with t [s], initial q, bias [rad/s], P, and sampled gyro rate.
%       Optional mag/sun and magI/sunI are row-oriented vector histories.
%       Optional validity, freshness, timestamp and tuning overrides inject faults.
% Outputs:
%   s - State histories and per-vector acceptance/NIS diagnostics.
load_system(fullfile(projectRoot(),'models','aocs_plant.slx'));
plantCleanup=onCleanup(@() close_system('aocs_plant',0));
m='MekfFixtureHarness'; new_system(m);
cleanup=onCleanup(@() close_system(m,0));
set_param(m,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','0.1','StopTime',num2str(f.t(end),17),'ReturnWorkspaceOutputs','on');
e=[m '/Estimator']; add_block('aocs_plant/GNC/State Estimation (MEKF)',e);
add_block('aocs_plant/GNC/GNC Tick',[m '/Tick']); add_line(m,'Tick/1','Estimator/Trigger');
ws=get_param(m,'ModelWorkspace'); n=numel(f.t);
triad=Simulink.Bus.createMATLABStruct('AttitudeInitializationBus');
triad.q_BI=f.q; triad.valid=1; triad.solution_time_s=0;
triad.DCM_BI=quat2dcm(f.q');
constantBus(m,ws,'TRIAD','AttitudeInitializationBus',triad);
triadSource='TRIAD/1';
if isfield(f,'triadValid')
    sampleBus(m,ws,'TRIAD Samples','TRIAD',triad,{'valid'},{f.triadValid},f.t);
    triadSource='TRIAD Samples/1';
end
reference=Simulink.Bus.createMATLABStruct('ReferenceVectorBus');
report=Simulink.Bus.createMATLABStruct('SensorReportBus');
status=Simulink.Bus.createMATLABStruct('SensorReadStatusBus');
status.Gyro.has_data=true;
status.Magnetometer.has_data=isfield(f,'mag');
status.CoarseSunSensors.has_data=isfield(f,'sun');
constantBus(m,ws,'References','ReferenceVectorBus',reference);
constantBus(m,ws,'Reports','SensorReportBus',report);
constantBus(m,ws,'Status','SensorReadStatusBus',status);
reportFields={'Gyro.omega_rad_s','Gyro.receive_time_s','Gyro.valid', ...
    'Magnetometer.B_B_T','Magnetometer.receive_time_s','Magnetometer.valid', ...
    'CoarseSunSensors.sun_B_unit','CoarseSunSensors.receive_time_s','CoarseSunSensors.valid'};
reportData={f.rate,get(f,'receipt',f.t),get(f,'valid',true(n,1)), ...
    get(f,'mag',zeros(n,3)),get(f,'magTime',f.t),get(f,'magValid',true(n,1)), ...
    get(f,'sun',zeros(n,3)),get(f,'sunTime',f.t),get(f,'sunValid',true(n,1))};
statusFields={'Gyro.new_data','Gyro.overrun','Magnetometer.new_data','CoarseSunSensors.new_data'};
statusData={get(f,'fresh',true(n,1)),get(f,'overrun',false(n,1)), ...
    get(f,'magFresh',true(n,1)),get(f,'sunFresh',true(n,1))};
refFields={'B_I_T','sun_I_unit','valid','sgp4_update_time_s'};
refData={get(f,'magI',zeros(n,3)),get(f,'sunI',zeros(n,3)), ...
    get(f,'refValid',true(n,1)),get(f,'refTime',f.t)};
sampleBus(m,ws,'Report Samples','Reports',report,reportFields,reportData,f.t);
sampleBus(m,ws,'Status Samples','Status',status,statusFields,statusData,f.t);
sampleBus(m,ws,'Reference Samples','References',reference,refFields,refData,f.t);
add_line(m,triadSource,'Estimator/1'); add_line(m,'Reference Samples/1','Estimator/2');
add_line(m,'Report Samples/1','Estimator/3'); add_line(m,'Status Samples/1','Estimator/4');
add_block('simulink/Signal Routing/Bus Selector',[m '/Observe'], ...
    'OutputSignals','q_BI,P_error,omega_BI_B_rad_s,gyro_bias_B_rad_s,valid,update_time_s');
add_line(m,'Estimator/1','Observe/1');
names={'q','P','rate','bias','valid','time'};
for k=1:numel(names), sink(m,names{k},['Observe/' num2str(k)]); end
add_block('simulink/Signal Routing/Bus Selector',[m '/Observe Health'], ...
    'OutputSignals', ['mode_id,control_usable,attitude_std_max_rad,' ...
    'state_age_s,correction_age_s,vector_update_accepted']);
add_line(m,'Estimator/2','Observe Health/1');
healthNames={'healthMode','controlUsable','attitudeStd','stateAge', ...
    'correctionAge','vectorAccepted'};
for k=1:numel(healthNames), sink(m,healthNames{k},['Observe Health/' num2str(k)]); end
sink(e,'magAccepted','Magnetometer Update/2'); sink(e,'magNIS','Magnetometer Update/3');
sink(e,'sunAccepted','Sun Vector Update/2'); sink(e,'sunNIS','Sun Vector Update/3');
cfg=evalin('base','AOCS_GNCConfig'); payload=cfg.Value;
payload.MEKF.initial_bias_B_rad_s=f.bias; payload.MEKF.initial_covariance=f.P;
overrides=get(f,'tuning',struct()); names=fieldnames(overrides);
for k=1:numel(names), payload.MEKF.(names{k})=overrides.(names{k}); end
overrides=get(f,'health',struct()); names=fieldnames(overrides);
for k=1:numel(names), payload.AttitudeHealth.(names{k})=overrides.(names{k}); end
cfg=Simulink.Parameter(payload); cfg.DataType='Bus: GNCConfigBus';
input=Simulink.SimulationInput(m); input=input.setVariable('AOCS_GNCConfig',cfg);
o=sim(input);
s.q=reshape(o.q.Data,4,[]); s.P=reshape(o.P.Data,6,6,[]);
s.rate=reshape(o.rate.Data,3,[]); s.bias=reshape(o.bias.Data,3,[]);
s.valid=o.valid.Data(:); s.time=o.time.Data(:);
s.magAccepted=o.magAccepted.Data(:); s.sunAccepted=o.sunAccepted.Data(:);
s.magNIS=o.magNIS.Data(:); s.sunNIS=o.sunNIS.Data(:);
s.healthMode=o.healthMode.Data(:); s.controlUsable=o.controlUsable.Data(:);
s.attitudeStd=o.attitudeStd.Data(:); s.stateAge=o.stateAge.Data(:);
s.correctionAge=o.correctionAge.Data(:); s.vectorAccepted=o.vectorAccepted.Data(:);
end

function value=get(f,name,fallback)
% Description:
%   Supplies a fixture default without modifying the caller's data.
% Arguments:
%   f, name, fallback - Fixture, optional field and default.
% Outputs:
%   value - Requested field or default.
if isfield(f,name), value=f.(name); else, value=fallback; end
end

function sink(m,name,source)
% Description:
%   Records one signal as a timeseries in simulation output.
% Arguments:
%   m, name, source - System, log variable and source port.
% Outputs:
%   None.
add_block('simulink/Sinks/To Workspace',[m '/' name], ...
    'VariableName',name,'SaveFormat','Timeseries');
add_line(m,source,[name '/1']);
end

function constantBus(m,ws,name,type,value)
% Description:
%   Creates an explicitly typed constant bus in the isolated harness workspace.
% Arguments:
%   m, ws, name, type, value - System, workspace, block, bus type and payload.
% Outputs:
%   None.
assignin(ws,[name 'Value'],value);
add_block('simulink/Sources/Constant',[m '/' name], ...
    'Value',[name 'Value'],'OutDataTypeStr',['Bus: ' type]);
end

function sampleBus(m,ws,name,source,payload,fields,data,t)
% Description:
%   Overrides selected bus fields with sampled, correctly typed signals.
% Arguments:
%   m, ws, name, source - Harness system/workspace, assignment and input bus.
%   payload, fields, data, t - Typed prototype, field names, histories, seconds.
% Outputs:
%   None.
add_block('simulink/Signal Routing/Bus Assignment',[m '/' name], ...
    'AssignedSignals',strjoin(fields,','));
add_line(m,[source '/1'],[name '/1']);
for k=1:numel(fields)
    value=payload; tokens=strsplit(fields{k},'.');
    for j=1:numel(tokens), value=value.(tokens{j}); end
    type=class(value); if islogical(value), type='boolean'; end
    sample(m,ws,[source num2str(k)],t,data{k},[name '/' num2str(k+1)],type);
end
end

function sample(m,ws,name,t,data,destination,type)
% Description:
%   Connects scalar/column samples; nonfinite fault rows bypass From Workspace.
% Arguments:
%   m, ws, name, t, data - Harness, workspace, source name, seconds, row samples.
%   destination, type - Destination port and Simulink type.
% Outputs:
%   None.
bad=any(~isfinite(data),2);
if any(bad)
    badValue=data(find(bad,1),:)';
    assert(isequaln(data(bad,:),repmat(badValue',sum(bad),1)), ...
        'Nonfinite fault rows must share the same injected value.');
    assignin(ws,[name 'BadValue'],badValue);
    assignin(ws,[name 'BadSamples'],[t double(bad)]); data(bad,:)=0;
end
assignin(ws,[name 'Samples'],[t double(data)]);
add_block('simulink/Sources/From Workspace',[m '/' name ' Input'], ...
    'VariableName',[name 'Samples'],'Interpolate','off','OutputAfterFinalValue','Holding final value');
add_block('simulink/Signal Attributes/Data Type Conversion',[m '/' name ' Type'],'OutDataTypeStr',type);
add_line(m,[name ' Input/1'],[name ' Type/1']); source=[name ' Type/1'];
if size(data,2)>1
    add_block('simulink/Math Operations/Reshape',[m '/' name ' Column'], ...
        'OutputDimensionality','Customize','OutputDimensions',sprintf('[%d 1]',size(data,2)));
    add_line(m,source,[name ' Column/1']); source=[name ' Column/1'];
end
if any(bad)
    add_block('simulink/Sources/Constant',[m '/' name ' Nonfinite'], ...
        'Value',[name 'BadValue'],'VectorParams1D','off');
    add_block('simulink/Sources/From Workspace',[m '/' name ' Inject'], ...
        'VariableName',[name 'BadSamples'],'Interpolate','off','OutputAfterFinalValue','Holding final value');
    add_block('simulink/Signal Routing/Switch',[m '/' name ' Fault'],'Criteria','u2 ~= 0');
    add_line(m,[name ' Nonfinite/1'],[name ' Fault/1']);
    add_line(m,[name ' Inject/1'],[name ' Fault/2']);
    add_line(m,source,[name ' Fault/3']); source=[name ' Fault/1'];
end
add_line(m,source,destination);
end
