function geometry = analyzeReferenceGeometry(out,AOCS,runDirectory)
% Diagnose orbit and reference geometry without another plant simulation.
	% Positions and field components are inertial ICRF; C_EI maps ICRF to ECEF.
	logs = out.logsout;
	t = unique([0:30:AOCS.Sim.StopTime_s,AOCS.Sim.StopTime_s]).';
	rTrue = rows(logs.get('r_I_m').Values,3,t);
	bLogged = rows(logs.get('B_I_T').Values,3,t);
	cfg = AOCS.GNCConfig.AttitudeInitialization;
	eop = AOCS.Environment.EarthOrientation;
	rOnboard = zeros(numel(t),3);
	bOnboard = zeros(size(rOnboard));
	bReconstructed = zeros(size(rOnboard));
	jd = AOCS.Epoch.TdbJulianDate+t/86400;
	sunPosition = 1000*planetEphemeris(jd,'Earth','Sun','405', ...
		'StartDate',juliandate(2020,1,1),'EndDate',juliandate(2030,1,1));
	for k = 1:numel(t)
		s = propagateOnboardSgp4([t(k);cfg.epoch_utc_jd;cfg.sgp4_semi_major_axis_m; ...
			cfg.sgp4_eccentricity;cfg.sgp4_inclination_deg;cfg.sgp4_raan_deg; ...
			cfg.sgp4_argument_of_periapsis_deg;cfg.sgp4_true_anomaly_deg;cfg.sgp4_bstar]);
		assert(s(7)==1,'Onboard orbit propagation failed.');
		rOnboard(k,:) = s(1:3).';
		utc = datevec(datetime(cfg.epoch_utc_jd,'ConvertFrom','juliandate','TimeZone','UTC')+seconds(t(k)));
		C_EI = dcmeci2ecef('IAU-2000/2006',utc,eop.DeltaAT_s, ...
			eop.DeltaUT1_s,reshape(eop.PolarMotion_rad,1,2),'dCIP',reshape(eop.DCIP_rad,1,2));
		bOnboard(k,:) = field(rOnboard(k,:),utc,C_EI,eop,cfg.epoch_decimal_year);
		bReconstructed(k,:) = field(rTrue(k,:),utc,C_EI,eop,cfg.epoch_decimal_year);
	end
	sunTrue = sunPosition-rTrue;
	positionError_km = vecnorm(rOnboard-rTrue,2,2)/1000;
	orbitDirectionError_deg = angle(rOnboard,rTrue);
	magneticDirectionError_deg = angle(bOnboard,bLogged);
	magneticReconstructionError_deg = angle(bReconstructed,bLogged);
	sunMagneticSeparation_deg = angle(sunTrue,bLogged);
	health = logs.get('AttitudeHealth').Values;
	healthSigma_deg = rad2deg(rows(health.attitude_std_max_rad,1,t));
	geometry = table(t,positionError_km,orbitDirectionError_deg, ...
		magneticDirectionError_deg,magneticReconstructionError_deg,sunMagneticSeparation_deg,healthSigma_deg);
	writetable(geometry,fullfile(runDirectory,'reference_geometry_30s.csv'));
	save(fullfile(runDirectory,'reference_geometry.mat'),'geometry','rOnboard','rTrue', ...
		'bOnboard','bLogged','bReconstructed','sunTrue');

	fprintf('Field reconstruction max direction difference: %.6g deg\n',max(magneticReconstructionError_deg));
	% Compare the reference defect with the estimator error, without retuning.
	qTrue = rows(logs.get('q_be').Values,4,t);
	estimate = logs.get('AttitudeEstimate').Values;
	qEstimated = rows(estimate.q_BI,4,t);
	qTrue = qTrue./vecnorm(qTrue,2,2);
	qEstimated = qEstimated./vecnorm(qEstimated,2,2);
	estimationError_deg = 2*acosd(min(1,abs(sum(qTrue.*qEstimated,2))));
	fig = figure('Name','Orbit and magnetic reference consistency','Color','w', ...
		'DefaultAxesColor','w','DefaultAxesXColor',[.15 .15 .15], ...
		'DefaultAxesYColor',[.15 .15 .15],'DefaultTextColor',[.15 .15 .15], ...
		'Position',[100 80 1100 850]);
	tiledlayout(3,1,'TileSpacing','compact');
	nexttile; plot(t,positionError_km); grid on;
	ylabel('Orbit difference [km]');
	nexttile; plot(t,magneticDirectionError_deg,t,estimationError_deg); grid on;
	ylabel('Direction / attitude [deg]');
	legend('Magnetic reference error','MEKF attitude error','Location','northoutside','Orientation','horizontal');
	nexttile; plot(t,sunMagneticSeparation_deg); grid on;
	ylabel('Sun-B separation [deg]'); xlabel('Time [s]');
	set(findall(fig,'Type','legend'),'Color','w','TextColor',[.15 .15 .15]);
	exportgraphics(fig,fullfile(runDirectory,'reference_geometry.png'),'Resolution',160);

end

function b = field(r,utc,C_EI,eop,year)
	lla = eci2lla(r,utc,'IAU-2000/2006',eop.DeltaAT_s,eop.DeltaUT1_s, ...
		reshape(eop.PolarMotion_rad,1,2),'dCIP',reshape(eop.DCIP_rad,1,2));
	bNED = igrfmagm(lla(3),lla(1),lla(2),year,14);
	p = lla(1); l = lla(2);
	C_NE = [-sind(p)*cosd(l),-sind(p)*sind(l),cosd(p); ...
		-sind(l),cosd(l),0; -cosd(p)*cosd(l),-cosd(p)*sind(l),-sind(p)];
	b = (C_EI.'*C_NE.'*bNED(:)).'*1e-9;
end

function value = angle(a,b)
	a = a./vecnorm(a,2,2); b = b./vecnorm(b,2,2);
	value = acosd(max(-1,min(1,sum(a.*b,2))));
end

function value = rows(ts,n,t)
	x = double(loggedSignalMatrix(ts.Data,n,ts.Name));
	[time,idx] = unique(ts.Time(:),'last');
	value = interp1(time,x(idx,:),t,'linear');
end
