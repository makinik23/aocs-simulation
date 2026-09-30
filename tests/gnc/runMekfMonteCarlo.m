function report = runMekfMonteCarlo(options)
% Description:
%   Runs seeded trials against the saved MEKF or the complete plant model.
%   Truth is used only for offline scoring in both scopes.
%
% Arguments:
%   options.Seeds - Integer seeds. Default is 1:6.
%   options.Duration_s - Trial duration [s], a multiple of 0.1. Default 60.
%   options.Scope - "isolated" MEKF fixture or "plant" end-to-end model.
%   options.Scenarios - Isolated scope only: "matched", "nominal", "outages".
%   options.SaveFile - Optional MAT report path. Default does not save.
%
% Outputs:
%   report - Reproducible per-trial metrics, configuration and aggregate data.
arguments
    options.Seeds (1,:) double {mustBeInteger,mustBePositive} = 1:6
    options.Duration_s (1,1) double {mustBePositive} = 60
    options.Scope (1,1) string {mustBeMember(options.Scope,["isolated","plant"])} = "isolated"
    options.Scenarios (1,:) string {mustBeMember(options.Scenarios,["matched","nominal","outages"])} = ["nominal","outages"]
    options.SaveFile (1,1) string = ""
end

if options.Scope=="plant"
    report=runPlantTrials(options);
    return
end

dt=0.1;

assert(abs(options.Duration_s/dt-round(options.Duration_s/dt))<1e-9, ...
    'AOCS:MonteCarlo:Duration','Duration_s must be a multiple of 0.1 s.');
assert(options.Duration_s>10 && ...
    (~any(options.Scenarios=="outages") || options.Duration_s>=50), ...
    'AOCS:MonteCarlo:Duration', ...
    'Duration_s must exceed 10 s, and outage trials require at least 50 s.');
setupAocsPaths([],true);
setupAocsSimulation('',true);
gnc=evalin('base','AOCS_GNCConfig');
tuning=gnc.Value.MEKF;
sensor=evalin('base','AOCS_SensorConfig');
physical=struct( ...
    'GyroNoiseDensity_rad_s_sqrt_Hz',sensor.Value.Gyro.noise_density_rad_s_sqrt_Hz, ...
    'GyroBiasRandomWalk_rad_s_sqrt_s',sensor.Value.Gyro.bias_random_walk_std_rad_s_sqrt_s, ...
    'GyroInitialBias_rad_s',sensor.Value.Gyro.bias_initial_rad_s(:), ...
    'MagDirectionNoiseStd_rad',deg2rad(0.45), ...
    'SunDirectionNoiseStd_rad',deg2rad(0.15), ...
    'MagFixedErrorStd_rad',deg2rad(0.35), ...
    'SunFixedErrorStd_rad',deg2rad(0.08));

scenarioCount=numel(options.Scenarios);
trialCount=numel(options.Seeds)*scenarioCount;
trials=repmat(emptyTrial(),trialCount,1);
index=0;

for scenario=options.Scenarios
    for seed=options.Seeds
        index=index+1;
        [fixture,truth]=makeTrial(seed,scenario,options.Duration_s,tuning,physical);
        output=simulateMekfFixture(fixture);
        trials(index)=scoreTrial(output,truth,fixture,seed,scenario);
        fprintf('MEKF MC %-8s seed %d: p95 %.3f deg, final bias %.3g rad/s, usable %.1f%%\n', ...
            scenario,seed,trials(index).AttitudeP95_deg, ...
            trials(index).FinalBiasError_rad_s,100*trials(index).ControlUsableFraction);
    end
end

report=struct();

report.Scope="isolated";
report.Seeds=options.Seeds;
report.Scenarios=options.Scenarios;
report.Duration_s=options.Duration_s;
report.SampleTime_s=dt;
report.SettlingTime_s=10;
report.FalseSafeThreshold_deg=5;
report.Tuning=tuning;
report.TruthModel=physical;
report.MatchedTruthModel=struct('MagDirectionNoiseStd_rad', ...
    tuning.magnetometer_direction_std_rad,'SunDirectionNoiseStd_rad', ...
    tuning.css_direction_std_rad,'FixedDirectionErrorStd_rad',0);
report.Trials=trials;
report.Summary=struct( ...
    'MaximumAttitudeP95_deg',max([trials.AttitudeP95_deg]), ...
    'MaximumFinalBiasError_rad_s',max([trials.FinalBiasError_rad_s]), ...
    'FalseSafeSamples',sum([trials.FalseSafeSamples]), ...
    'FalseSafeSamples2deg',sum([trials.FalseSafeSamples2deg]), ...
    'MaximumUsableAttitudeError_deg',max([trials.MaximumUsableAttitudeError_deg]), ...
    'MinimumCovarianceEigenvalue',min([trials.MinimumCovarianceEigenvalue]), ...
    'AllInitialized',all([trials.Initialized]));

if options.SaveFile~=""
    folder=fileparts(options.SaveFile);
    if folder~="" && ~isfolder(folder), mkdir(folder); end
    save(options.SaveFile,'report');
end
end

function report=runPlantTrials(options)
% Description:
%   Repeats the complete plant-to-GNC model with independent sensor RNG seeds.
% Arguments:
%   options - Seeds, duration and optional MAT report path.
% Outputs:
%   report - Per-seed attitude, bias and health metrics.
setupAocsPaths([],true);
assert(options.Duration_s>10,'AOCS:MonteCarlo:Duration', ...
    'Duration_s must exceed the 10 s settling interval.');
[baseInput,AOCS]=createAocsSimulationInput();
cleanup=onCleanup(@() close_system(AOCS.Model.Name,0));
variables=baseInput.Variables;
sensor=variables(strcmp({variables.Name},'AOCS_SensorConfig')).Value;
gnc=variables(strcmp({variables.Name},'AOCS_GNCConfig')).Value;
trials=repmat(struct('Seed',0,'AttitudeP95_deg',NaN, ...
    'MaximumAttitudeError_deg',NaN,'FinalBiasError_rad_s',NaN, ...
    'Initialized',false,'FinalControlUsable',false, ...
    'FalseSafeSamples',0),numel(options.Seeds),1);

for k=1:numel(options.Seeds)
    seed=options.Seeds(k);
    physical=sensor.Value;
    physical.Gyro.noise_seed=30000+10*seed+1;
    physical.Gyro.bias_seed=30000+10*seed+2;
    physical.Magnetometer.noise_seed=30000+10*seed+3;
    physical.Magnetometer.bias_seed=30000+10*seed+4;
    physical.CoarseSunSensors.noise_seed=30000+10*seed+5;
    parameter=Simulink.Parameter(physical);
    parameter.DataType='Bus: SensorConfigBus';
    input=baseInput.setVariable('AOCS_SensorConfig',parameter);
    input=input.setModelParameter('StopTime',num2str(options.Duration_s,17));
    output=sim(input);
    estimate=output.logsout.get('AttitudeEstimate').Values;
    health=output.logsout.get('AttitudeHealth').Values;
    truth=extractAocsState(output);
    time=estimate.q_BI.Time(:);
    estimatedQ=reshape(estimate.q_BI.Data,4,[])';
    trueQ=interp1(truth.q_be.Time(:), ...
        loggedSignalMatrix(truth.q_be.Data,4,'q_be'),time,'linear');
    trueQ=trueQ./vecnorm(trueQ,2,2);
    angle=2*acosd(min(1,abs(sum(estimatedQ.*trueQ,2))));
    trueBias=output.get('GyroTrueBias');
    assert(isa(trueBias,'timeseries'),'AOCS:MonteCarlo:MissingBiasTruth', ...
        'The full plant did not log gyroscope bias truth.');
    bias=reshape(estimate.gyro_bias_B_rad_s.Data,3,[])';
    biasTruth=interp1(trueBias.Time(:), ...
        loggedSignalMatrix(trueBias.Data,3,'GyroTrueBias'),time,'linear');
    settled=time>=10;
    usable=logical(health.control_usable.Data(:));
    trials(k).Seed=seed;
    trials(k).AttitudeP95_deg=percentile(angle(settled),95);
    trials(k).MaximumAttitudeError_deg=max(angle(settled));
    trials(k).FinalBiasError_rad_s=norm(bias(end,:)-biasTruth(end,:));
    trials(k).Initialized=all(estimate.initialized.Data(settled));
    trials(k).FinalControlUsable=usable(end);
    trials(k).FalseSafeSamples=nnz(settled & usable & angle>5);
    fprintf('MEKF plant MC seed %d: p95 %.3f deg, final bias %.3g rad/s\n', ...
        seed,trials(k).AttitudeP95_deg,trials(k).FinalBiasError_rad_s);
end

report=struct('Scope',"plant",'Seeds',options.Seeds, ...
    'Duration_s',options.Duration_s, ...
    'SettlingTime_s',10,'FalseSafeThreshold_deg',5, ...
    'Trials',trials,'SensorSeedFormula',"30000+10*seed+[1..5]", ...
    'BaselineSensorConfig',sensor.Value,'GNCConfig',gnc.Value);

if options.SaveFile~=""
    folder=fileparts(options.SaveFile);
    if folder~="" && ~isfolder(folder), mkdir(folder); end
    save(options.SaveFile,'report');
end
end

function trial=emptyTrial()
% Description:
%   Fixes report fields before the first Simulink run.
% Arguments:
%   None.
% Outputs:
%   trial - Empty scalar trial metrics.
trial=struct('Seed',0,'Scenario',"",'Initialized',false, ...
    'AttitudeP95_deg',NaN,'AttitudeMaximum_deg',NaN, ...
    'FinalAttitudeError_deg',NaN,'FinalBiasError_rad_s',NaN, ...
    'BiasErrorP95_rad_s',NaN,'NEESMean',NaN,'NEESP95',NaN, ...
    'AttitudeNEESMean',NaN,'BiasNEESMean',NaN, ...
    'MagNISMean',NaN,'SunNISMean',NaN,'MagAcceptedFraction',NaN, ...
    'SunAcceptedFraction',NaN,'ControlUsableFraction',NaN, ...
    'FalseSafeSamples',0,'FalseSafeSamples2deg',0, ...
    'MaximumUsableAttitudeError_deg',NaN,'AttitudeStdMean_deg',NaN, ...
    'LostSamples',0,'MinimumCovarianceEigenvalue',NaN, ...
    'CovarianceFinite',false,'GyroOutageGuarded',false, ...
    'VectorOutageGuarded',false,'OutlierRejected',false);
end

function [f,truth]=makeTrial(seed,scenario,duration,tuning,physical)
% Description:
%   Draws independent rate, bias, vector noise, and outage histories.
% Arguments:
%   seed, scenario, duration, tuning, physical - Reproduction key and models.
% Outputs:
%   f - Public MEKF bus histories; truth - offline truth histories.
stream=RandStream('mt19937ar','Seed',seed);
f.t=(0:0.1:duration)'; n=numel(f.t); dt=0.1;
f.bias=tuning.initial_bias_B_rad_s(:);
f.P=tuning.initial_covariance;
q0=rotationQuaternion(0.7*randn(stream,3,1));
rate=[0.018;0.012;-0.009]+0.003*randn(stream,3,1);
f.q=quatmultiply(q0',rotationQuaternion(deg2rad(2)*randn(stream,3,1))')';
magI=[0.3;0.1;0.5]; magI=magI/norm(magI);
sunI=[-0.2;0.9;0.2]; sunI=sunI/norm(sunI);
f.magI=repmat((4e-5*magI)',n,1);
f.sunI=repmat(sunI',n,1);
f.mag=zeros(n,3); f.sun=zeros(n,3); f.rate=zeros(n,3);
truth.q=zeros(4,n); truth.bias=zeros(3,n);
truth.bias(:,1)=physical.GyroInitialBias_rad_s+1e-4*randn(stream,3,1);
gyroSigma=physical.GyroNoiseDensity_rad_s_sqrt_Hz/sqrt(dt);
magSigma=physical.MagDirectionNoiseStd_rad;
sunSigma=physical.SunDirectionNoiseStd_rad;
magFixed=physical.MagFixedErrorStd_rad*randn(stream,3,1);
sunFixed=physical.SunFixedErrorStd_rad*randn(stream,3,1);

if scenario=="matched"
    magSigma=tuning.magnetometer_direction_std_rad;
    sunSigma=tuning.css_direction_std_rad;
    magFixed=zeros(3,1); sunFixed=zeros(3,1);
end

for k=1:n
    if k>1
        truth.bias(:,k)=truth.bias(:,k-1)+ ...
            physical.GyroBiasRandomWalk_rad_s_sqrt_s*sqrt(dt)*randn(stream,3,1);
    end

    truth.q(:,k)=quatmultiply(q0',rotationQuaternion(rate*f.t(k))')';
    C=quat2dcm(truth.q(:,k)');
    f.rate(k,:)=rate'+truth.bias(:,k)'+gyroSigma*randn(stream,1,3);
    magnetic=C*magI; sunlight=C*sunI;
    f.mag(k,:)=4e-5*noisyDirection( ...
        magnetic+cross(magFixed,magnetic),magSigma,stream)';
    f.sun(k,:)=noisyDirection( ...
        sunlight+cross(sunFixed,sunlight),sunSigma,stream)';
end

f.fresh=true(n,1); f.magFresh=true(n,1); f.sunFresh=true(n,1);

if scenario=="outages"
    f.sunFresh(f.t>=15 & f.t<30)=false;
    f.magFresh(f.t>=24 & f.t<32)=false;
    f.fresh(f.t>=38 & f.t<40)=false;
    f.mag(f.t>=48 & f.t<49,:)=-f.mag(f.t>=48 & f.t<49,:);
end

truth.rate=rate;
truth.gyroOutage=f.t>=38 & f.t<40;
end

function direction=noisyDirection(direction,sigma,stream)
% Description:
%   Perturbs a unit direction in its tangent plane, then renormalizes it.
% Arguments:
%   direction, sigma, stream - Unit direction, angular std [rad], RNG stream.
% Outputs:
%   direction - Perturbed unit direction.
direction=direction/norm(direction);
noise=randn(stream,3,1);
noise=noise-direction*(direction'*noise);
direction=direction+sigma*noise;
direction=direction/norm(direction);
end

function trial=scoreTrial(s,truth,f,seed,scenario)
% Description:
%   Scores posterior error, covariance, innovation and health against truth.
% Arguments:
%   s, truth, f, seed, scenario - Output, offline truth and reproduction key.
% Outputs:
%   trial - Scalar score record.
trial=emptyTrial(); trial.Seed=seed; trial.Scenario=scenario;
n=numel(f.t);
assert(numel(s.valid)==n && size(s.P,3)==n, ...
    'AOCS:MonteCarlo:Length','MEKF output does not cover every GNC tick.');
settled=f.t>=10;
trial.Initialized=all(s.valid(settled & f.fresh));
dotProduct=abs(sum(s.q.*truth.q,1));
attitude=2*acosd(min(1,max(0,dotProduct)))';
biasError=vecnorm(s.bias-truth.bias,2,1)';
trial.AttitudeP95_deg=percentile(attitude(settled),95);
trial.AttitudeMaximum_deg=max(attitude(settled));
trial.FinalAttitudeError_deg=attitude(end);
trial.FinalBiasError_rad_s=biasError(end);
trial.BiasErrorP95_rad_s=percentile(biasError(settled),95);
trial.MagNISMean=mean(s.magNIS(settled & logical(s.magAccepted)));
trial.SunNISMean=mean(s.sunNIS(settled & logical(s.sunAccepted)));
trial.MagAcceptedFraction=mean(s.magAccepted(settled));
trial.SunAcceptedFraction=mean(s.sunAccepted(settled));
trial.ControlUsableFraction=mean(s.controlUsable(settled));
trial.FalseSafeSamples=nnz(settled & logical(s.controlUsable) & attitude>5);
trial.FalseSafeSamples2deg=nnz(settled & logical(s.controlUsable) & attitude>2);
usable=settled & logical(s.controlUsable);

if any(usable), trial.MaximumUsableAttitudeError_deg=max(attitude(usable)); end

trial.AttitudeStdMean_deg=mean(rad2deg(s.attitudeStd(settled)));
trial.LostSamples=nnz(s.healthMode==3);
staleGyro=truth.gyroOutage & s.stateAge>=0.2;
trial.GyroOutageGuarded=any(staleGyro) && all(~s.controlUsable(staleGyro));
vectorOutage=f.t>=24 & f.t<30 & s.correctionAge>5+1e-6;
trial.VectorOutageGuarded=any(vectorOutage) && all(~s.controlUsable(vectorOutage));
outlier=f.t>=48 & f.t<49;
trial.OutlierRejected=any(outlier) && all(~s.magAccepted(outlier));
trial.CovarianceFinite=all(isfinite(s.P),'all');
minimum=Inf; nees=nan(n,1); attitudeNees=nan(n,1); biasNees=nan(n,1);

for k=1:n
    P=s.P(:,:,k);
    if any(~isfinite(P),'all'), minimum=NaN; continue; end
    minimum=min(minimum,min(eig((P+P')/2)));
    if ~settled(k), continue; end
    [R,flag]=chol((P+P')/2);
    if flag~=0, continue; end
    delta=quatmultiply(quatconj(s.q(:,k)'),truth.q(:,k)');
    if delta(1)<0, delta=-delta; end
    vector=delta(2:4)'; length=norm(vector);
    if length<1e-12
        angle=2*vector;
    else
        angle=2*atan2(length,delta(1))*vector/length;
    end
    error=[angle;truth.bias(:,k)-s.bias(:,k)];
    whitened=R'\error;
    nees(k)=whitened'*whitened;
    attitudeWhitened=chol(P(1:3,1:3))'\angle;
    biasWhitened=chol(P(4:6,4:6))'\error(4:6);
    attitudeNees(k)=attitudeWhitened'*attitudeWhitened;
    biasNees(k)=biasWhitened'*biasWhitened;
end

trial.MinimumCovarianceEigenvalue=minimum;
trial.NEESMean=mean(nees,'omitnan');
trial.NEESP95=percentile(nees(isfinite(nees)),95);
trial.AttitudeNEESMean=mean(attitudeNees,'omitnan');
trial.BiasNEESMean=mean(biasNees,'omitnan');

end

function value=percentile(data,percent)
% Description:
%   Computes a linearly interpolated empirical percentile without toolboxes.
% Arguments:
%   data, percent - Finite samples and percentile in [0, 100].
% Outputs:
%   value - Empirical percentile, or NaN for an empty sample.
data=sort(data(:));

if isempty(data), value=NaN; return; end

position=1+(numel(data)-1)*percent/100;
lo=floor(position); hi=ceil(position);
value=data(lo)+(position-lo)*(data(hi)-data(lo));
end

function q=rotationQuaternion(vector)
% Description:
%   Constructs an independent scalar-first Hamilton exponential.
% Arguments:
%   vector - Rotation vector [rad].
% Outputs:
%   q - Unit quaternion.
angle=norm(vector);

if angle<1e-14, q=[1;0.5*vector]; else, q=[cos(angle/2);sin(angle/2)*vector/angle]; end

end
