function simIn = setAocsParameters(simIn, target, varargin)
%SETAOCSPARAMETERS Share parameter definitions across interactive and batch use.
if isempty(simIn)
    set_param(target, varargin{:});
elseif string(target) == string(simIn.ModelName)
    simIn = simIn.setModelParameter(varargin{:});
else
    for k = 1:2:numel(varargin)
        simIn = simIn.setBlockParameter(target, varargin{k}, varargin{k+1});
    end
end
end
