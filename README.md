# Electrical Grid Protection and GFL Stability Benchmarking

This repository now contains two complementary tracks:

1. **Protection relay testing artifacts** (OMICRON reports for differential, distance, and overcurrent relays).
2. **Machine-learning-assisted GFL converter stability benchmarking** with strict fairness constraints across tuning methods.

---

## Contents
- [A) Relay Test Artifacts](#a-relay-test-artifacts)
- [B) GFL Stability Study (ML + Fair Comparison)](#b-gfl-stability-study-ml--fair-comparison)
- [Stability Criteria Definition (Concretized)](#stability-criteria-definition-concretized)
- [Fairness Protocol](#fairness-protocol)
- [Outputs](#outputs)

---

## A) Relay Test Artifacts
Available report files:
- `OCP_relay.pdf`
- `Distance Protection report.pdf`
- `differential protection report.pdf`
- `Distance_Testing.occ`
- `OCP_Relay_report.occ`
- `differential_Protection.occ`

These documents capture classical protection testing workflows using OMICRON CMC platforms.

---

## B) GFL Stability Study (ML + Fair Comparison)
Main script:
- `GFL_STABILITY_STUDY_v6.m`

### What v6 improves
- **Same dataset for all methods**: baseline, linear tuning, lookup-table, and GPR all use the same grid case pool and candidate controller set.
- **Concretized stability criterion** with explicit post-event, final-value, smoothing, and settling windows.
- **Hard/soft split**: hard pass/fail constraints gate stability; soft weighted score ranks stable solutions.
- **Additional stability guard**: optional RoCoF hard constraint.
- **Many more figures**: 10 figure outputs for rate, score maps, criterion margins, fairness checks, and parameter landscapes.

---

## Stability Criteria Definition (Concretized)
The script enforces the following hard criteria (all must pass):

- **H1 current limit**: `I_peak_pu <= 1.15`
- **H2 transient frequency bound**: `max(|f(t)-50|) <= 1.0 Hz`
- **H3 final frequency recovery**: `|mean(f_last_0.1s)-50| <= 0.05 Hz`
- **H4 damping minimum**: `zeta >= 0.02`
- **H5 settling time**: `T_settle <= 2.0 s`
- **H6 RoCoF** *(optional, enabled by default in v6)*: `max(|df/dt|) <= 2.0 Hz/s`

### Windowing policy used everywhere
- Post-step analysis starts at `T_stp + 20 ms`.
- Final steady-state estimate uses last `0.1 s`.
- Settling uses a strict “remain in band afterward” check.
- Smoothing is based on 2 grid cycles.

These rules remove ambiguity and keep all methods directly comparable.

---

## Fairness Protocol
To avoid partial comparison:

1. Build one **oracle dataset** by sweeping the exact same controller candidate grid for every `(SCR, R/X)` case.
2. Fit all methods using this identical trainable subset.
3. Evaluate all fitted methods on the exact same full case grid.
4. Export casewise results (`CSV`) and summary tables for transparent auditing.

---

## Outputs
Running `GFL_STABILITY_STUDY_v6.m` generates:

- `results_v6/results_casewise_v6.csv`
- `results_v6/results_summary_v6.csv`
- `results_v6/*.png` (10 figures)
- `results_v6/GFL_study_v6_<timestamp>.mat`

> Note: Running the script requires MATLAB/Simulink with the model `GFL_LCL_WeakGrid_AI` and expected logged signals (`is_abc_n1` or `is_abc`, and `omega`).

