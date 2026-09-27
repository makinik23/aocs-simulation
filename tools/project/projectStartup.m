function projectStartup()
%PROJECTSTARTUP Initialize paths and schema; never load a scenario or compile.
setupAocsPaths();
configureAocsFileGeneration();
createAocsBuses();
end
