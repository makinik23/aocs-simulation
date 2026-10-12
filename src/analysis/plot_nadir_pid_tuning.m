function files = plot_nadir_pid_tuning(selectedLabel)
%PLOT_NADIR_PID_TUNING Export the recorded acquisition comparison and integral.
arguments
    selectedLabel (1,1) string
end
root=setupAocsPaths(); directory=fullfile(root,'docs','data','nadir_pid_tuning');
history=jsondecode(fileread(fullfile(directory,'history.json')));
baseline=jsondecode(fileread(fullfile(directory,'baseline_before_tuning.json')));
directories={baseline.RunDirectory}; labels={'T00: before tuning'};
for k=1:numel(history)
    h=history(k);
    if strcmp(h.Status,'completed') && h.InitialError_deg==30 && h.StopTime_s==1200
        directories{end+1}=h.RunDirectory; labels{end+1}=h.Label; %#ok<AGROW>
    end
end
fig=lightFigure('PID pole-placement tuning',[100 100 1100 800]);
tiledlayout(2,1,'TileSpacing','compact');
for panel=1:2
    nexttile; hold on;
    for k=1:numel(directories)
        data=load(fullfile(root,directories{k},'nadir_report.mat'),'report'); r=data.report;
        plot(r.Time_s,r.TrueError_deg,'LineWidth',1.3);
    end
    grid on; ylabel('True attitude error [deg]'); xlabel('Time [s]');
    if panel==1
        title('PID only: same 30 deg initial error, noise seeds and torque limits');
        legend(labels,'Interpreter','none','Location','northeast');
    else
        ylim([0 4]); xlim([100 1200]); yline(1,':');
        title('Acquisition rebound and subsequent holding');
    end
end
files.Comparison=fullfile(directory,'comparison.png');
exportgraphics(fig,files.Comparison,'Resolution',160);
selected=history(strcmp({history.Label},selectedLabel)); assert(isscalar(selected),'Select exactly one trial label.');
data=load(fullfile(root,selected.RunDirectory,'simulation.mat'),'out','AOCS');
xi=data.out.logsout.get('pid_integral_acceleration_rad_s2').Values;
integralTorque=-loggedSignalMatrix(xi.Data,3,'xi')*data.AOCS.Spacecraft.I_B';
torque=data.out.logsout.get('tau_control_B_Nm').Values;
applied=loggedSignalMatrix(torque.Data,3,'torque');
fig=lightFigure('Selected PID integral and applied torque',[100 100 1100 800]);
tiledlayout(2,1,'TileSpacing','compact');
nexttile; plot(xi.Time,1e6*integralTorque,'LineWidth',1.2); grid on;
ylabel('Integral torque [microNm]'); legend('x_B','y_B','z_B');
title(['Selected PID: ' char(selectedLabel)],'Interpreter','none');
nexttile; plot(torque.Time,1e6*applied,'LineWidth',1.1); grid on;
yline(1e6*data.AOCS.Control.MaxTorque_Nm,':'); yline(-1e6*data.AOCS.Control.MaxTorque_Nm,':');
ylabel('Applied total torque [microNm]'); xlabel('Time [s]'); legend('x_B','y_B','z_B');
files.Integral=fullfile(directory,'integral_and_torque.png');
exportgraphics(fig,files.Integral,'Resolution',160);
end

function fig=lightFigure(name,position)
fig=figure('Name',name,'Color','w','Position',position, ...
    'DefaultAxesColor','w','DefaultAxesXColor',[.15 .15 .15], ...
    'DefaultAxesYColor',[.15 .15 .15],'DefaultTextColor',[.15 .15 .15], ...
    'DefaultLegendColor','w','DefaultLegendTextColor',[.15 .15 .15]);
end
