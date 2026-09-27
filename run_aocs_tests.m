function [results, reportDirectory] = run_aocs_tests(profile)
%RUN_AOCS_TESTS Reproducible suites with JUnit, JSON, and console diagnostics.
% core excludes native atmosphere and external-data validation.
% full includes all tests; unavailable optional validations are reported skipped.
% validation requires the Planet fixture and executes the full suite.
arguments
    profile (1,1) string {mustBeMember(profile,["core","full","validation"])} = "core"
end
root = setupAocsPaths([], true);
configureAocsFileGeneration();
import matlab.unittest.TestSuite
import matlab.unittest.TestRunner
import matlab.unittest.plugins.XMLPlugin
if profile == "core"
    folders = ["tests/sensors", "tests/gnc", "tests/infrastructure"];
    suite = TestSuite.fromFolder(fullfile(root, folders(1)));
    for folder = folders(2:end)
        suite = [suite, TestSuite.fromFolder(fullfile(root, folder))]; %#ok<AGROW>
    end
    files = ["AtmosphereProductsTest", "Dtm2020InputsTest", ...
        "SentmanPanelAerodynamicsTest", "SolarRadiationPressureTest"];
    for file = files
        suite = [suite, TestSuite.fromFile(fullfile(root, 'tests', 'environment', file + '.m'))]; %#ok<AGROW>
    end
else
    requireDtm2020Native();
    if profile == "validation" && ~isfile(fullfile(root, 'validation', 'planet', ...
            'data', 'planet_dove_oem_reference.mat'))
        error("AOCS:Validation:MissingPlanetFixture", ...
            "Required Planet fixture missing. See validation/planet/README.md.");
    end
    suite = TestSuite.fromFolder(fullfile(root, 'tests'), IncludingSubfolders=true);
end
base = fullfile(root, 'test-results');
if ~isfolder(base)
    mkdir(base);
end
reportDirectory = string(tempname(base));
mkdir(reportDirectory);
previousDiary = get(0, 'Diary');
previousDiaryFile = get(0, 'DiaryFile');
diary(fullfile(reportDirectory, 'console.log'));
diaryCleanup = onCleanup(@() restoreDiary(previousDiary, previousDiaryFile));
runner = TestRunner.withTextOutput;
runner.addPlugin(XMLPlugin.producingJUnitFormat(fullfile(reportDirectory, 'junit.xml')));
runner.addPlugin(matlab.unittest.plugins.DiagnosticsRecordingPlugin);
results = runner.run(suite);
report = struct('Profile', profile, 'Provenance', aocsRunProvenance(root), ...
    'Passed', nnz([results.Passed]), 'Failed', nnz([results.Failed]), ...
    'Skipped', nnz([results.Incomplete] & ~[results.Failed]));
report.Tests = struct('Name', {results.Name}, 'Passed', num2cell([results.Passed]), ...
    'Failed', num2cell([results.Failed]), 'Incomplete', num2cell([results.Incomplete]), ...
    'Duration_s', num2cell([results.Duration]));
writeAocsJson(fullfile(reportDirectory, 'summary.json'), report);
save(fullfile(reportDirectory, 'results.mat'), 'results');
fprintf('Tests: %d passed, %d failed, %d skipped. Reports: %s\n', ...
    report.Passed, report.Failed, report.Skipped, reportDirectory);
if any([results.Failed])
    error("AOCS:Tests:Failed", "%d test(s) failed; see %s", report.Failed, reportDirectory);
end
end

function restoreDiary(state, file)
diary off;
set(0, 'DiaryFile', file);
set(0, 'Diary', state);
end
