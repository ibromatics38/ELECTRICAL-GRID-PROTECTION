# Textbook-Style Report: GFL Inverter Stability Benchmark and GPR-Based Tuning

## 1. Introduction
This report explains the complete implementation in `GFL_STABILITY_STUDY_REFACTORED.m`, which was designed to benchmark multiple controller tuning strategies for a grid-following (GFL) inverter under weak-grid conditions.

The implementation has one principal goal: **fair, physically meaningful, and reproducible comparison** of tuning policies (Baseline, linear heuristics, lookup table, and Gaussian Process Regression) on exactly the same operating points.

---

## 2. Problem Statement and Design Objectives
The script addresses four practical research requirements:

1. **Equal dataset across all methods**  
   All strategies are evaluated on a shared `(SCR, R/X)` dataset.
2. **Physically interpretable stability definition**  
   Hard pass/fail constraints are separated from soft quality scoring.
3. **Data-driven optimization with safety constraints**  
   GPR predicts tuning parameters but remains bounded and bandwidth-feasible.
4. **Simulink-centric reproducibility**  
   The full workflow is executable from one script and writes reproducible outputs.

---

## 3. Overall Pipeline Architecture
The script executes six conceptual stages:

1. Build a shared operating dataset from all SCR and R/X combinations.
2. Sweep candidate controller knobs (`taus`, `tspll`) on each operating point.
3. Compute stability score for each candidate by hard + soft criteria.
4. Select the best stable candidate per operating point and train GPR models.
5. Evaluate all strategies on the same dataset.
6. Summarize and persist training, benchmark, and aggregate outputs.

This flow is intentionally unidirectional (dataset -> training -> modeling -> evaluation) to avoid leakage or hidden retuning during comparison.

---

## 4. Configuration and Shared Dataset
The implementation defines:

- model name and disturbance settings (`DipPct`, `T_stp`, `SimTime`),
- SCR grid and R/X grid,
- controller sweep ranges,
- hard criteria thresholds,
- soft score weights,
- strategy list.

### 4.1 Why shared dataset matters
Benchmark fairness requires each strategy to face **the same difficulty distribution**. If strategy A is tested on easier points (e.g., larger SCR) and strategy B on harder points, reported gains are not meaningful.

The script enforces identical cases using a cartesian product table:

\[
\mathcal{D} = \{(\text{SCR}_i, (R/X)_j)\}_{i=1..N_{SCR},\, j=1..N_{RX}}
\]

with `15 x 7 = 105` operating points.

---

## 5. Controller Knobs and Physical Bandwidth Constraint
Two key tuning knobs are used:

- `taus`: current controller time constant,
- `tspll`: PLL settling-related time constant.

A bandwidth separation rule is enforced:

\[
\omega_{PLL} \le \alpha\,\omega_{CI}
\]

where

\[
\omega_{CI}=\frac{1}{\tau_s}, \quad \omega_{PLL}=\frac{4}{0.707\,t_{s,PLL}}
\]

and `alpha_bw = 0.20` by default.

### Why this is physically important
In weak grids, an overly fast PLL can couple destabilizingly with current dynamics. Forcing PLL bandwidth lower than current-loop bandwidth is a classical decoupling heuristic that increases robustness.

---

## 6. Training Data Generation Logic
For each `(SCR, R/X)` case:

1. Iterate all feasible `(taus, tspll)` sweep pairs.
2. Apply grid and gains to parameter struct.
3. Simulate in Simulink.
4. Evaluate stability + quality score.
5. Record one row in `training_table`.
6. Keep the **best-scoring** feasible point as `best_knobs` for that case.

This produces:

- a dense training table of candidate outcomes,
- one optimal (or least-bad fallback) tuning per operating point.

---

## 7. Gaussian Process Regression (GPR) Layer
The script trains two independent regressors:

- `GPR_taus : X -> taus`
- `GPR_tspll : X -> tspll`

with feature map

\[
X_f = [SCR, R/X, \log_{10}(SCR), \log_{10}(\max(R/X, 0.01)), SCR\cdot R/X]
\]

and ARD squared-exponential kernels.

### Why this feature map
- raw terms represent first-order behavior,
- logs better linearize sensitivity near weak-grid region,
- interaction term captures cross-coupling (`SCR * R/X`).

### Fallback philosophy
If stable samples are too few, models are left empty and runtime falls back to baseline constants. This avoids pretending to have learned from insufficient data.

---

## 8. Strategy Evaluation Protocol
Each strategy generates `(taus, tspll)` for every case:

- Baseline: fixed constants,
- LinearPLL: SCR-based PLL interpolation,
- LinearBoth: SCR-based interpolation for both knobs,
- LookupTable: discrete region map in `(SCR, R/X)`,
- GPR_AI: model prediction then safety clipping.

After generation, all tunings are clipped to valid bounds and re-checked against bandwidth feasibility.

### Critical fairness feature
All strategies call the same simulator and the same `stability_assessment` function, so differences in outcomes are attributable to tuning policy rather than evaluation method.

---

## 9. Stability Criteria Framework
The implementation separates **hard criteria** (binary admissibility) from **soft criteria** (quality ranking among admissible cases).

## 9.1 Hard Criteria (must all pass)
- H1 Current limit: `I_peak_pu <= 1.15`
- H2 Transient frequency bound: `max|f-50| <= 1.0 Hz`
- H3 Final frequency recovery: `|f_final-50| <= 0.05 Hz`
- H4 Damping floor: `zeta >= 0.02`
- H5 Settling speed: `T_settle <= 2.0 s`

Overall stability is

\[
\text{Stable} = H1 \land H2 \land H3 \land H4 \land H5
\]

## 9.2 Soft Criteria (only computed when hard pass)
- S1 current quality,
- S2 frequency quality,
- S3 damping margin quality,
- S4 settling quality.

Weighted score:

\[
Score = w_1 S1 + w_2 S2 + w_3 S3 + w_4 S4
\]

with default weights `[0.25, 0.30, 0.25, 0.20]`.

### Why hard + soft is better than one monolithic scalar
A single scalar alone may hide standards violations. Hard gates enforce compliance first; soft scores then rank performance among compliant cases.

---

## 10. Signal Processing and Metric Extraction
The stability evaluator uses `logsout` signals:

- `is_abc_n1` for current RMS envelope,
- `omega` for frequency,
- optional `vdq` compatibility pattern retained.

A post-disturbance window starts at `T_stp + 20 ms` to skip immediate switching artifacts.

### 10.1 Current metrics
- Compute RMS from three-phase current.
- Smooth by moving average over roughly two cycles.
- Extract `I_peak`, `I_final`, `I_ratio`, and H1.

### 10.2 Frequency metrics
- Convert `omega` to Hz.
- Evaluate max deviation and final deviation for H2/H3.

### 10.3 Damping estimate
A conservative estimate is used:

1. logarithmic decrement on peaks (when available),
2. amplitude decay between first/second half windows,
3. choose the more conservative (smaller) positive damping estimate.

This prevents over-optimistic damping claims when oscillation evidence is mixed.

### 10.4 Settling time
Settling is defined as the first time after which smoothed current remains within ±5% of final value.

---

## 11. Data Structures and Output Artifacts
The implementation relies on preallocated tables to improve reliability and traceability:

- `training_table`: per-candidate sweep outcomes,
- `results`: per-case per-strategy benchmark outputs,
- `summary`: strategy-level aggregate statistics.

Files written to `results_refactored/`:

- `training_<timestamp>.csv`
- `results_<timestamp>.csv`
- `summary_<timestamp>.csv`
- `study_<timestamp>.mat`

This enables reproducible post-processing without rerunning all simulations.

---

## 12. Critical Review: Strengths and Remaining Gaps
## 12.1 Strengths
- Fair dataset governance across strategies.
- Physically informed hard criteria and PLL/current bandwidth separation.
- Conservative damping treatment.
- Robust fallbacks when learning data are sparse.
- Single-script reproducible workflow.

## 12.2 Remaining gaps (for publication-grade rigor)
1. **Damping proxy limitation**: current-envelope damping is indirect; a small-signal eigenvalue damping ratio would be stronger evidence.
2. **Single disturbance type**: only one dip profile is currently embedded; include depth/duration diversity for stronger claims.
3. **No uncertainty bands in final report**: GPR predictive uncertainty could be propagated into confidence intervals.
4. **Signal-name coupling**: strict dependence on `logsout` naming; a signal adapter layer would improve portability.

---

## 13. Recommended Next Enhancements
1. Add `criteria_profile` presets (strict/standard/exploratory).
2. Add k-fold or leave-one-SCR-out validation for GPR generalization.
3. Add Monte Carlo over measurement noise and parameter mismatch.
4. Add eigenvalue-based damping cross-check block for selected points.
5. Add automatic report generation plotting Pareto fronts: stability rate vs mean score.

---

## 14. How to Use in Practice
1. Open MATLAB/Simulink environment with model `GFL_LCL_WeakGrid_AI` available.
2. Ensure `logsout` contains `is_abc_n1` and `omega`.
3. Run `GFL_STABILITY_STUDY_REFACTORED.m`.
4. Inspect CSV/MAT artifacts in `results_refactored/`.
5. Compare strategy outcomes in `summary_*.csv` and inspect failure regions in `results_*.csv`.

---

## 15. Conclusion
The refactored implementation establishes a strong benchmark foundation: equal-case testing, explicit physical stability gates, and safe data-driven tuning. It is appropriate for systematic engineering comparison and can be extended to publication-grade evidence by adding eigenvalue-level damping validation and uncertainty quantification.
