# Development tools

`tools/` contains repository maintenance utilities. It is not flight software
and is not part of the simulated onboard execution path.

| Folder | Responsibility |
|---|---|
| `project/` | Project startup, path-independent root lookup, generated-file settings and cleanup |
| `diagnostics/` | Read-only environment and Simulink guideline checks |
| `provenance/` | Hashing, Git/run metadata, shell quoting and JSON report writing |
| `native/` | DTM2020 native build, build signature and readiness checks |
| `model_builders/` | Optional colors, fonts, and previews through `styleAocsModel` |
| `visualization/` | Local visualization adapters; Basilisk files remain Git-ignored |

Run `setupAocsPaths` before invoking a tool outside the MATLAB Project. The
production model is edited directly in Simulink. `styleAocsModel` deliberately
does not move blocks, resize them, or route signal lines; those choices belong to
the user and remain stored in `models/aocs_plant.slx`.
