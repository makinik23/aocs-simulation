function rootDirectory = projectRoot()
%PROJECTROOT Return the absolute path to the repository root.

rootDirectory = string(fileparts(mfilename("fullpath")));
end
