function plotNadirTrial(t,traces,tauExternal,outputFolder)
% Separate columns compare acquisition from 5 and 30 degrees.
	fig = figure('Name','Nadir PID: constant torque and PDF Lyapunov function', ...
		'Tag','NadirGuidance','Color','w','Position',[80 80 1250 950]);
	layout = tiledlayout(fig,4,numel(traces),'TileSpacing','compact','Padding','compact');
	colors = lines(3);
	for k = 1:numel(traces)
		trace = traces(k);
		nexttile(layout,k);
		semilogy(t,max(trace.angle_deg,1e-12),'LineWidth',1.3);
		yline(0.1,'--','0.1 deg');
		grid on;
		ylabel('Attitude error [deg]');
		title(sprintf('Initial error: %g deg',trace.initialAngle_deg));

		nexttile(layout,numel(traces)+k);
		colororder(colors);
		plot(t,1e6*trace.tauIntegral,'LineWidth',1.2);
		hold on;
		for j = 1:3
			yline(-1e6*tauExternal(j),'--','Color',colors(j,:),'HandleVisibility','off');
		end
		grid on;
		ylabel('Integral torque [\muNm]');
		legend('X','Y','Z','Location','best');

		nexttile(layout,2*numel(traces)+k);
		semilogy(t,max(trace.pdf.V,realmin),'LineWidth',1.3);
		grid on;
		ylabel('V from PDF');

		nexttile(layout,3*numel(traces)+k);
		plot(t(1:end-1),trace.pdf.deltaV,'LineWidth',1.1);
		yline(0,'k:');
		grid on;
		ylabel('\Delta V from PDF');
		xlabel('Time [s]');
		fprintf('PDF V, %g deg: V0=%.6g, Vend=%.6g, max DeltaV=%.6g\n', ...
			trace.initialAngle_deg,trace.pdf.V(1),trace.pdf.V(end),max(trace.pdf.deltaV));
	end
	title(layout,'Nadir tracking | constant body torque | unchanged PID gains');
	subtitle(layout,'Dashed torque lines: required compensation. PDF V is evaluated on the sampled nonlinear trajectory.');
	exportgraphics(fig,fullfile(outputFolder,'nadir_constant_torque.png'),'Resolution',160);
end
