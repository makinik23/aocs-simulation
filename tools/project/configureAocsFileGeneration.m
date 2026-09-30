function configureAocsFileGeneration()
%CONFIGUREAOCSFILEGENERATION Keep generated Simulink files under build/.
% These are session settings; global MATLAB preferences are not changed.
root = projectRoot();
cache = fullfile(root, 'build', 'simulink', 'cache');
code = fullfile(root, 'build', 'simulink', 'codegen');
current = Simulink.fileGenControl('getConfig');
if string(current.CacheFolder) ~= cache || string(current.CodeGenFolder) ~= code
    Simulink.fileGenControl('set', 'CacheFolder', char(cache), ...
        'CodeGenFolder', char(code), 'createDir', true);
end
end
