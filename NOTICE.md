# Attribution

This is a dataset-adapted evaluator, not an official NAVSIM benchmark release.

`src/etri_pdms/comfort.py` derives from NAVSIM v1.1 `pdm_comfort_metrics.py`. The StateIndex import was adapted to the local state representation. NAVSIM's Apache 2.0 license is included at `licenses/NAVSIM_LICENSE`.

- NAVSIM scorer: https://github.com/autonomousvision/navsim/blob/v1.1/navsim/planning/simulation/planner/pdm_planner/scoring/pdm_scorer.py
- NAVSIM comfort: https://github.com/autonomousvision/navsim/blob/v1.1/navsim/planning/simulation/planner/pdm_planner/scoring/pdm_comfort_metrics.py
- nuPlan ahead/behind/stopped-track semantics: https://github.com/motional/nuplan-devkit/blob/master/nuplan/planning/simulation/observation/idm/utils.py
- VAD displacement semantics: https://github.com/hustvl/VAD/blob/main/projects/mmdet3d_plugin/VAD/VAD.py
- Viser: https://viser.studio/

The Frenet MPC with a soft single CLF was adapted from the MPC-CLF code supplied for this project. Time-indexed references, speed-state dynamics and failure handling were added for evaluation. The original standalone demonstration is not redistributed here.

Dataset and model licenses remain separate. This source repository includes no ETRI/nuScenes dataset files, trained weights, or actual model predictions. Synthetic fixtures are generated locally by `pdms demo` and the test suite.
