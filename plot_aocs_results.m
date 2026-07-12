function figures = plot_aocs_results(varargin)
% Description:
%   Compatibility wrapper for the renamed attitude plotting entrypoint.
%
% Arguments:
%   varargin - Forwarded to plot_attitude_results.
%
% Outputs:
%   figures - Handles returned by plot_attitude_results.

figures = plot_attitude_results(varargin{:});
end
