classdef OrbitNavigationIntegrationTest < matlab.unittest.TestCase
	% Verify navigation-fed references and controller startup in the full plant.

	properties (TestParameter)
		controlCase = struct( ...
			'PD', struct('Name', "PD", 'ActiveLog', "control_active"), ...
			'PID', struct('Name', "PID", 'ActiveLog', "pid_control_active"))
	end

	methods (TestClassSetup)
		function prepareProject(testCase)
			root = fileparts(fileparts(fileparts(mfilename("fullpath"))));
			testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
			setupAocsPaths(root, true);
		end
	end

	methods (Test, TestTags = ["Integration", "FullPlant"])
		function referencesShareNavigationAndWaitForGnss(testCase, controlCase)
			[out, aocs] = run_nadir_pointing_demo( ...
				InitialError_deg=5, StopTime_s=35, MakePlots=false, ...
				Controller=controlCase.Name);
			nav = out.OrbitNavigationStateLog;
			reference = out.logsout.get('ReferenceVectors').Values;
			orbit = out.OrbitReferenceLog;
			t = nav.valid.Time(:);
			acquisition = aocs.Sensors.GNSS.AcquisitionTime_s;
			before = t < acquisition;
			after = t >= acquisition;
			active = out.logsout.get(controlCase.ActiveLog).Values;
			torque = out.logsout.get('tau_control_B_Nm').Values;
			torqueRows = loggedSignalMatrix(torque.Data, 3, torque.Name);
			alpha = out.logsout.get('alpha_ref_DI_D_rad_s2').Values;
			alphaRows = loggedSignalMatrix(alpha.Data, 3, alpha.Name);
			firstReady = find(alpha.Time >= acquisition, 1);
			q = out.logsout.get('q_ref_DI').Values;
			qRows = loggedSignalMatrix(q.Data, 4, q.Name);
			omega = out.logsout.get('omega_ref_DI_D_rad_s').Values;
			omegaRows = loggedSignalMatrix(omega.Data, 3, omega.Name);
			position = loggedSignalMatrix(nav.position_I_m.Data, 3, 'r');
			velocity = loggedSignalMatrix(nav.velocity_I_m_s.Data, 3, 'v');
			[expectedQ, expectedOmega] = guidanceReference(position(after, :), ...
				velocity(after, :));
			qDot = abs(sum(qRows(after, :) .* expectedQ, 2));

			testCase.verifyTrue(any(after));
			testCase.verifyFalse(any(nav.valid.Data(before)));
			testCase.verifyTrue(all(nav.valid.Data(after)));
			testCase.verifyFalse(any(active.Data(active.Time < acquisition)));
			testCase.verifyTrue(any(active.Data(active.Time >= acquisition)));
			testCase.verifyEqual(torqueRows(torque.Time < acquisition, :), ...
				zeros(nnz(torque.Time < acquisition), 3), AbsTol=1e-12);
			testCase.verifyTrue(all(isfinite(torqueRows(:))));
			testCase.verifyTrue(all(isfinite(alphaRows(:))));
			testCase.verifyEqual(qDot, ones(nnz(after), 1), AbsTol=1e-12);
			testCase.verifyEqual(omegaRows(after, :), expectedOmega, AbsTol=1e-12);
			testCase.verifyEqual(alphaRows(firstReady, :), zeros(1, 3), AbsTol=1e-12);
			testCase.verifyEqual(reference.orbit_solution_id.Data(:), ...
				nav.solution_id.Data(:));
			testCase.verifyEqual(reference.orbit_update_time_s.Data(after), ...
				nav.epoch_s.Data(after), AbsTol=1e-9);
			testCase.verifyEqual(loggedSignalMatrix(orbit.r_I_m.Data, 3, 'r'), ...
				loggedSignalMatrix(nav.position_I_m.Data, 3, 'r'), AbsTol=1e-9);
			testCase.verifyEqual(loggedSignalMatrix(orbit.v_I_m_s.Data, 3, 'v'), ...
				loggedSignalMatrix(nav.velocity_I_m_s.Data, 3, 'v'), AbsTol=1e-9);
			testCase.verifyEqual(logical(out.NavigationReferenceReadyLog.Data(:)), ...
				logical(nav.valid.Data(:)));
		end
	end
end

function [q, omega] = guidanceReference(position, velocity)
	% Compute the LVLH reference from navigation geometry.

	n = size(position, 1);
	q = zeros(n, 4);
	omega = zeros(n, 3);

	for index = 1:n
		r = position(index, :).';
		v = velocity(index, :).';
		h = cross(r, v);
		z = -r / norm(r);
		y = -h / norm(h);
		x = cross(y, z);
		q(index, :) = dcm2quat([x.'; y.'; z.']);
		omega(index, :) = [0, -norm(h) / dot(r, r), 0];
	end
end
