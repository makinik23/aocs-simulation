function rows = sampledRows(logs, name, width, t)
	signal = logs.get(name);
	assert(isa(signal, 'Simulink.SimulationData.Signal'), ...
		'Expected one logged signal named %s.', name);
	ts = signal.Values;

	if ts.IsTimeFirst
		data = reshape(ts.Data, numel(ts.Time), width);
	else
		data = reshape(ts.Data, width, numel(ts.Time)).';
	end

	% Match actual sample instants. Previous-value interpolation can select
	% the preceding tick when floating-point timestamps differ slightly.
	[sampleTimes, keep] = unique(ts.Time(:), 'last');
	indices = interp1(sampleTimes, keep, t, 'nearest', NaN);
	assert(all(isfinite(indices)), 'Log %s does not cover the requested times.', name);
	assert(all(abs(ts.Time(indices) - t) < 1e-8), ...
		'Log %s has no samples at the requested controller ticks.', name);

	rows = data(indices,:);
	assert(all(isfinite(rows), 'all'), 'Log %s contains nonfinite values.', name);
end

