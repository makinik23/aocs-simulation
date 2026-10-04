# Development tools

`tools/` contains repository maintenance utilities.

| Folder | Responsibility |
|---|---|
| `project/` | Project startup, path-independent root lookup, generated-file settings and cleanup |
| `diagnostics/` | Read only environment and Simulink guideline checks |
| `provenance/` | Hashing, Git/run metadata, shell quoting and JSON report writing |
| `native/` | DTM2020 native build, build signature and readiness checks |
| `model_builders/` | Optional colors, fonts, and previews through `styleAocsModel` |

Run `setupAocsPaths` before invoking a tool outside the MATLAB Project.
