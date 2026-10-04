function report = plot_nadir_pointing_results(out,AOCS,options)
%PLOT_NADIR_POINTING_RESULTS True/estimated tracking of the onboard LVLH.
% This measures tracking of the onboard SGP4 reference, not orbit accuracy.
arguments
    out
    AOCS struct
    options.MakePlots (1,1) logical = true
end
logs = out.logsout;
ref = logs.get('q_ref_DI').Values;
t = ref.Time(:);
qRef = rows(ref,4);
truth = logs.get('q_be').Values;
qTrue = resample(truth,4,t,'linear');
estimate = logs.get('AttitudeEstimate').Values;
qEst = resample(estimate.q_BI,4,t,'previous');
qRef = qRef./vecnorm(qRef,2,2);
qTrue = qTrue./vecnorm(qTrue,2,2);
qEst = qEst./vecnorm(qEst,2,2);
angleTrue = 2*acosd(min(1,abs(sum(qTrue.*qRef,2))));
angleEst = 2*acosd(min(1,abs(sum(qEst.*qRef,2))));
estimationError = 2*acosd(min(1,abs(sum(qEst.*qTrue,2))));
omegaTrue = resample(logs.get('omega_b').Values,3,t,'linear');
omegaRef = resample(logs.get('omega_ref_DI_D_rad_s').Values,3,t,'previous');
rateError = zeros(numel(t),3);
for k=1:numel(t)
    C_BD = quat2dcm(qTrue(k,:))*quat2dcm(qRef(k,:))';
    rateError(k,:) = omegaTrue(k,:) - (C_BD*omegaRef(k,:)')';
end
torque = logs.get('tau_control_B_Nm').Values;
active = logs.get('control_active').Values;
useIntegral = isfield(AOCS.Control,'UseIntegral') && AOCS.Control.UseIntegral;
controllerName = 'PD';
if useIntegral, controllerName = 'PID'; end
demandName = 'tau_demand_B_Nm';
if useIntegral, demandName = 'tau_pid_demand_B_Nm'; end
demand = logs.get(demandName).Values;
demandRows = rows(demand,3);
activeAtDemand = resample(active,1,demand.Time(:),'previous') > 0.5;
saturated = any(abs(demandRows) > AOCS.Control.MaxTorque_Nm + 1e-12,2) & activeAtDemand;
if any(activeAtDemand)
    saturationFraction = nnz(saturated)/nnz(activeAtDemand);
else
    saturationFraction = 0;
end
tail = t >= max(t(1),t(end)-60);
report = struct('InitialError_deg',angleTrue(1), ...
    'FinalError_deg',angleTrue(end), ...
    'TailMaxError_deg',max(angleTrue(tail)), ...
    'TailRmsError_deg',sqrt(mean(angleTrue(tail).^2)), ...
    'FinalEstimationError_deg',estimationError(end), ...
    'FinalRateError_deg_s',rad2deg(norm(rateError(end,:))), ...
    'TailTrueRateRms_deg_s',rad2deg(sqrt(mean(sum(rateError(tail,:).^2,2)))), ...
    'MaxAxisTorque_Nm',max(abs(rows(torque,3)),[],'all'), ...
    'MaxAxisDemandTorque_Nm',max(abs(demandRows),[],'all'), ...
    'SaturationFraction',saturationFraction, ...
    'Controller',controllerName, ...
    'ControlActiveFraction',mean(double(active.Data(:))), ...
    'Time_s',t,'TrueError_deg',angleTrue,'EstimatedError_deg',angleEst, ...
    'EstimationError_deg',estimationError);
fprintf('LVLH error: %.3f -> %.3f deg; last 60 s max %.3f deg, RMS %.3f deg.\n', ...
    report.InitialError_deg,report.FinalError_deg,report.TailMaxError_deg,report.TailRmsError_deg);
fprintf('Final rate error %.5f deg/s; final estimation error %.3f deg.\n', ...
    report.FinalRateError_deg_s,report.FinalEstimationError_deg);
fprintf('Saturation on %.2f%% of active GNC samples; max demand %.3f uNm, applied %.3f uNm.\n', ...
    100*saturationFraction,1e6*report.MaxAxisDemandTorque_Nm,1e6*report.MaxAxisTorque_Nm);
if ~options.MakePlots, return; end
fig = figure('Name',['Nadir pointing - quaternion ' controllerName],'Color','w', ...
    'Position',[100 80 1100 800],'DefaultAxesColor','w', ...
    'DefaultAxesXColor',[.15 .15 .15],'DefaultAxesYColor',[.15 .15 .15], ...
    'DefaultTextColor',[.15 .15 .15], ...
    'DefaultAxesColorOrder',[0 .447 .741; .85 .325 .098; .25 .55 .15]);
tiledlayout(4,1,'TileSpacing','compact');
nexttile; plot(t,angleTrue,t,angleEst,'--'); grid on;
ylabel('Attitude error [deg]'); legend('Truth vs reference','MEKF vs reference');
kp=AOCS.Control.Kp_s2; kd=AOCS.Control.Kd_s;
if useIntegral && isfield(AOCS.Control,'PIDKp_s2')
    kp=AOCS.Control.PIDKp_s2; kd=AOCS.Control.PIDKd_s;
end
caption=sprintf('Quaternion %s, Kp = %.4g s^{-2}, Kd = %.4g s^{-1}',controllerName,kp,kd);
if useIntegral, caption=[caption sprintf(', Ki = %.4g s^{-3}',AOCS.Control.Ki_s3)]; end
title(caption);
nexttile; plot(t,rad2deg(rateError)); grid on;
ylabel('Rate error [deg/s]'); legend('x_B','y_B','z_B');
nexttile; stairs(torque.Time,1e6*rows(torque,3)); grid on;
yline(1e6*AOCS.Control.MaxTorque_Nm,':'); yline(-1e6*AOCS.Control.MaxTorque_Nm,':');
ylabel('Applied torque [uN m]'); legend('x_B','y_B','z_B');
nexttile; stairs(active.Time,double(active.Data(:))); grid on; ylim([-0.1 1.1]);
ylabel('Control active'); xlabel('Time [s]');
set(findall(fig,'Type','legend'),'Color','w','TextColor',[.15 .15 .15]);
axesHandles = findall(fig,'Type','axes');
for k=1:numel(axesHandles)
    axesHandles(k).Toolbar.Visible = 'off';
end
end

function values = rows(ts,n)
values = loggedSignalMatrix(ts.Data,n,ts.Name);
end

function values = resample(ts,n,t,method)
[times,indices] = unique(ts.Time(:),'last');
data = double(rows(ts,n));
values = interp1(times,data(indices,:),t,method,'extrap');
end
