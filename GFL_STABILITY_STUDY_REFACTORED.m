%% GFL_STABILITY_STUDY_REFACTORED.m
% =========================================================================
% Refactored GFL inverter stability study with shared dataset + robust criteria
% =========================================================================
% Key upgrades over legacy versions:
%  1) Single shared dataset across all strategies (exact same SCR/RX cases).
%  2) Consistent criteria namespace (config.criteria / config.weights).
%  3) Vectorized/structured result handling (table preallocation, no struct growth).
%  4) Stability assessment with explicit hard criteria and conservative damping estimate.
%  5) Supports Simulink-first flow while keeping tuners modular.
%
% NOTE:
%  - This script assumes model signals exist in logsout with names:
%      'is_abc_n1', 'omega', optionally 'vdq'.
%  - If your signal names differ, update stability_assessment() extraction section.
% =========================================================================

clear; clc; close all;

%% ======================== CONFIGURATION ========================
config.modelName = 'GFL_LCL_WeakGrid_AI';
config.outputDir = fullfile(pwd, 'results_refactored');
if ~exist(config.outputDir, 'dir'), mkdir(config.outputDir); end

% Shared dataset definition (ALL methods evaluated on this exact set)
config.SCR_list = [0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 2, 2.5, 3, 4, 5, 7, 10];
config.RX_list  = [0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 3.0];

% Disturbance setup
config.DipPct = 20;
config.SimTime = 8;
config.T_stp = 4;

% Controller candidate sweep
config.taus_sweep  = [0.3e-3, 0.5e-3, 0.8e-3, 1e-3, 1.5e-3, 2e-3, 3e-3, 5e-3, 8e-3];
config.tspll_sweep = [0.012, 0.018, 0.025, 0.035, 0.050, 0.070, 0.100, 0.140, 0.200];
config.alpha_bw = 0.20; % omega_pll <= alpha_bw * omega_current_loop

% Hard criteria (must all pass)
config.criteria.I_limit_pu   = 1.15;
config.criteria.f_bound_Hz   = 1.0;
config.criteria.f_recovery_Hz = 0.05;
config.criteria.zeta_min     = 0.02;
config.criteria.T_settle_max = 2.0;

% Soft criteria weights (sum to 1)
config.weights.current   = 0.25;
config.weights.frequency = 0.30;
config.weights.damping   = 0.25;
config.weights.settling  = 0.20;

config.strategies = {'Baseline', 'LinearPLL', 'LinearBoth', 'LookupTable', 'GPR_AI'};

fprintf('=== Refactored GFL Stability Study ===\n');
fprintf('Dataset: %d SCR x %d RX = %d cases\n', numel(config.SCR_list), numel(config.RX_list), numel(config.SCR_list)*numel(config.RX_list));

%% ======================== SHARED DATASET ========================
dataset = build_dataset(config.SCR_list, config.RX_list);

%% ======================== TRAINING DATA (same dataset) ========================
[best_knobs, training_table] = generate_training_data(config, dataset);

%% ======================== TRAIN GPR ========================
gpr_models = train_gpr_models(best_knobs);

%% ======================== EVALUATION ========================
results = evaluate_all_strategies(config, dataset, gpr_models);

%% ======================== ANALYSIS + SAVE ========================
summary = summarize_results(results, config.strategies);
disp(summary);

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
writetable(training_table, fullfile(config.outputDir, ['training_' timestamp '.csv']));
writetable(results, fullfile(config.outputDir, ['results_' timestamp '.csv']));
writetable(summary, fullfile(config.outputDir, ['summary_' timestamp '.csv']));
save(fullfile(config.outputDir, ['study_' timestamp '.mat']), ...
    'config', 'dataset', 'best_knobs', 'training_table', 'gpr_models', 'results', 'summary');

fprintf('Saved outputs in: %s\n', config.outputDir);

%% ======================== FUNCTIONS ========================

function dataset = build_dataset(SCR_list, RX_list)
    [S, R] = ndgrid(SCR_list, RX_list);
    dataset = table(S(:), R(:), 'VariableNames', {'SCR','RX'});
end

function [best_knobs, training_table] = generate_training_data(config, dataset)
    nCases = height(dataset);
    nComb = numel(config.taus_sweep) * numel(config.tspll_sweep);

    train_capacity = nCases * nComb;
    train_arr = nan(train_capacity, 5); % [SCR RX taus tspll score]
    row_ptr = 0;

    best_knobs = table('Size', [nCases 6], ...
        'VariableTypes', {'double','double','double','double','double','logical'}, ...
        'VariableNames', {'SCR','RX','taus','tspll','score','isStable'});

    fprintf('Generating training data (%d cases)...\n', nCases);
    for k = 1:nCases
        SCR = dataset.SCR(k); RX = dataset.RX(k);
        best_score = -Inf; best_taus = NaN; best_tspll = NaN; best_stable = false;

        for taus = config.taus_sweep
            for tspll = config.tspll_sweep
                if ~bandwidth_feasible(taus, tspll, config.alpha_bw), continue; end

                p = params_base(config);
                p = apply_grid(p, SCR, RX);
                p.taus_n2 = taus;
                p.ts_pll_n2 = tspll;
                p = compute_gains(p);

                [simOut, ok] = run_sim(config.modelName, p);
                if ok
                    [score, metrics] = stability_assessment(simOut, p, config);
                else
                    score = 0;
                    metrics.isStable = false;
                end

                row_ptr = row_ptr + 1;
                train_arr(row_ptr,:) = [SCR, RX, taus, tspll, score];

                if score > best_score
                    best_score = score;
                    best_taus = taus;
                    best_tspll = tspll;
                    best_stable = metrics.isStable;
                end
            end
        end

        best_knobs{k, :} = {SCR, RX, best_taus, best_tspll, best_score, best_stable};
    end

    train_arr = train_arr(1:row_ptr, :);
    training_table = array2table(train_arr, 'VariableNames', {'SCR','RX','taus','tspll','score'});
end

function gpr_models = train_gpr_models(best_knobs)
    stable_rows = best_knobs.isStable;
    X = [best_knobs.SCR(stable_rows), best_knobs.RX(stable_rows)];
    y_taus = best_knobs.taus(stable_rows);
    y_tspll = best_knobs.tspll(stable_rows);

    gpr_models = struct('taus',[],'tspll',[]);
    if size(X,1) < 8
        warning('Not enough stable points for robust GPR. Falling back to defaults.');
        return;
    end

    Xf = feature_map(X);
    try
        gpr_models.taus = fitrgp(Xf, y_taus, 'KernelFunction', 'ardsquaredexponential', 'Standardize', true);
        gpr_models.tspll = fitrgp(Xf, y_tspll, 'KernelFunction', 'ardsquaredexponential', 'Standardize', true);
    catch ME
        warning('GPR training failed: %s', ME.message);
        gpr_models.taus = [];
        gpr_models.tspll = [];
    end
end

function results = evaluate_all_strategies(config, dataset, gpr_models)
    nCases = height(dataset);
    nStrat = numel(config.strategies);

    baseCols = {'SCR','RX'};
    stratCols = {};
    for i = 1:nStrat
        s = config.strategies{i};
        stratCols = [stratCols, ...
            {['taus_' s], ['tspll_' s], ['stable_' s], ['score_' s], ...
             ['Ipk_' s], ['fmax_' s], ['zeta_' s], ['Tset_' s]}]; %#ok<AGROW>
    end

    varNames = [baseCols, stratCols];
    varTypes = [repmat({'double'},1,2), repmat({'double','double','logical','double','double','double','double','double'},1,nStrat)];
    results = table('Size', [nCases numel(varNames)], 'VariableTypes', varTypes, 'VariableNames', varNames);

    for k = 1:nCases
        SCR = dataset.SCR(k); RX = dataset.RX(k);
        results.SCR(k) = SCR; results.RX(k) = RX;

        for i = 1:nStrat
            sName = config.strategies{i};
            [taus, tspll] = get_tuning(sName, SCR, RX, gpr_models, config.alpha_bw);

            p = params_base(config);
            p = apply_grid(p, SCR, RX);
            p.taus_n2 = taus; p.ts_pll_n2 = tspll;
            p = compute_gains(p);
            [simOut, ok] = run_sim(config.modelName, p);

            if ok
                [score, m] = stability_assessment(simOut, p, config);
            else
                score = 0;
                m = struct('isStable', false, 'I_peak_pu', NaN, 'f_max_dev_Hz', NaN, 'zeta', NaN, 'T_settle', NaN);
            end

            results.(['taus_' sName])(k) = taus;
            results.(['tspll_' sName])(k) = tspll;
            results.(['stable_' sName])(k) = m.isStable;
            results.(['score_' sName])(k) = score;
            results.(['Ipk_' sName])(k) = m.I_peak_pu;
            results.(['fmax_' sName])(k) = m.f_max_dev_Hz;
            results.(['zeta_' sName])(k) = m.zeta;
            results.(['Tset_' sName])(k) = m.T_settle;
        end
    end
end

function summary = summarize_results(results, strategies)
    nCases = height(results);
    n = numel(strategies);
    summary = table('Size', [n 4], ...
        'VariableTypes', {'string','double','double','double'}, ...
        'VariableNames', {'Strategy','StableCases','StableRatePct','MeanStableScore'});

    for i = 1:n
        s = strategies{i};
        stable = results.(['stable_' s]);
        scores = results.(['score_' s]);

        summary.Strategy(i) = string(s);
        summary.StableCases(i) = sum(stable);
        summary.StableRatePct(i) = 100 * sum(stable) / nCases;
        if any(stable)
            summary.MeanStableScore(i) = mean(scores(stable));
        else
            summary.MeanStableScore(i) = 0;
        end
    end
end

function Xf = feature_map(X)
    SCR = X(:,1);
    RX = X(:,2);
    Xf = [SCR, RX, log10(SCR), log10(max(RX,0.01)), SCR.*RX];
end

function tf = bandwidth_feasible(taus, tspll, alpha)
    omega_ci = 1/taus;
    omega_pll = 4/(0.707*tspll);
    tf = omega_pll <= alpha * omega_ci;
end

function [taus, tspll] = get_tuning(strategy, SCR, RX, gpr_models, alpha)
    switch lower(strategy)
        case 'baseline'
            taus = 1e-3; tspll = 0.018;
        case 'linearpll'
            a = max(0, min(1, (SCR - 0.8) / (10 - 0.8)));
            tspll = 0.200 + a*(0.012 - 0.200);
            taus = 1e-3;
        case 'linearboth'
            a = max(0, min(1, (SCR - 0.8) / (10 - 0.8)));
            tspll = 0.200 + a*(0.012 - 0.200);
            taus = 8e-3 + a*(0.5e-3 - 8e-3);
        case 'lookuptable'
            taus_tbl  = [5e-3,5e-3,8e-3; 3e-3,2e-3,3e-3; 1.5e-3,1e-3,1.5e-3; 0.5e-3,0.5e-3,0.8e-3];
            tspll_tbl = [0.18,0.18,0.20; 0.07,0.05,0.08; 0.035,0.025,0.04; 0.015,0.012,0.02];
            if SCR <= 1.5, sz = 1; elseif SCR <= 3, sz = 2; elseif SCR <= 6, sz = 3; else, sz = 4; end
            if RX <= 0.3, rz = 1; elseif RX <= 1, rz = 2; else, rz = 3; end
            taus = taus_tbl(sz,rz); tspll = tspll_tbl(sz,rz);
        case 'gpr_ai'
            if ~isempty(gpr_models.taus) && ~isempty(gpr_models.tspll)
                X = feature_map([SCR RX]);
                taus = predict(gpr_models.taus, X);
                tspll = predict(gpr_models.tspll, X);
            else
                taus = 1e-3; tspll = 0.018;
            end
        otherwise
            error('Unknown strategy: %s', strategy);
    end

    % Bound and enforce bandwidth rule
    taus = max(0.3e-3, min(taus, 10e-3));
    tspll = max(0.010, min(tspll, 0.30));
    if ~bandwidth_feasible(taus, tspll, alpha)
        omega_ci = 1/taus;
        tspll = 4 / (0.707 * alpha * omega_ci);
        tspll = max(0.010, min(tspll, 0.30));
    end
end

function p = params_base(config)
    p.Sn1 = 500e6; p.Un1 = 320e3; p.f1 = 50;
    p.w_g1 = 2*pi*p.f1; p.Vdcref_n1 = 640e3;
    p.np = 1; p.l_km = 50; p.Lpu = 1e-3; p.Rpu = 0.03; p.Vl = 220e3;
    p.Lac = p.Lpu * p.l_km * (p.Un1/p.Vl)^2;
    p.Rac = p.Rpu * p.l_km * (p.Un1/p.Vl)^2;
    p.Ts = 5e-5; p.Tsim = config.SimTime; p.T_stp = config.T_stp;
    p.plots_step = p.Ts; p.sat_current = 1.15;
    p.epsilon = 1; p.v_ini = 1e-5; p.magnitud_delay = p.Ts;
    p.Sn_n1 = p.Sn1; p.cosfi_n1 = 1; p.f_n1 = p.f1;
    p.w_n1 = 2*pi*p.f_n1; p.Un_n1 = p.Un1;
    p.Vpeak_n1 = p.Un_n1/sqrt(3)*sqrt(2);
    p.Xn_n1 = p.Un_n1^2/p.Sn_n1; p.Ln_n1 = p.Xn_n1/(2*pi*p.f_n1);
    p.Rc_n1 = 0.01*p.Xn_n1; p.Lc_n1 = 0.2*p.Ln_n1;
    p.Cac_n1 = (1/5.88)*(1/(p.w_n1*p.Xn_n1));
    p.vaini_n1 = p.Vpeak_n1; p.vbini_n1 = -0.5*p.Vpeak_n1;
    p.vcini_n1 = -0.5*p.Vpeak_n1; p.Q_ref_n1 = 10e6;
    p.ts_pll_n1 = 0.025; p.xi_pll_n1 = 0.707;
    omega_pll_n1 = 4/(p.ts_pll_n1*p.xi_pll_n1);
    p.Vpeak = p.Un1*sqrt(2/3);
    p.kp_pll1 = p.xi_pll_n1*2*omega_pll_n1/p.Vpeak;
    tau_pll_n1 = 2*p.xi_pll_n1/omega_pll_n1;
    p.ki_pll1 = p.kp_pll1/tau_pll_n1;
    p.Sn_n2 = p.Sn1; p.f_n2 = p.f1; p.w_n2 = 2*pi*p.f_n2;
    p.Un_n2 = p.Un1; p.Vpeak_n2 = p.Un_n2/sqrt(3)*sqrt(2);
    p.Inom_n2 = p.Sn_n2/p.Un_n2/sqrt(3);
    p.Xn_n2 = p.Un_n2^2/p.Sn_n2; p.Ln_n2 = p.Xn_n2/(2*pi*p.f_n2);
    p.Rc_n2 = 0.01*p.Xn_n2; p.Lc_n2 = 0.2*p.Ln_n2;
    p.Cac_n2 = (1/5.88)*(1/(p.w_n2*p.Xn_n2));
    p.Q_ref_n2 = 0; p.Pvsc0 = 200e6;
    p.taus_n2 = 1e-3; p.ts_pll_n2 = 0.018; p.xi_pll_n2 = 0.707;
    p.tau_u = 40e-3; p.tau_p = 0.2; p.tau_fp = 20e-3;
    p.tau_q = 0.2; p.tau_fq = 20e-3;
    p.k_droop_f_n2 = 1/0.05; p.tau_droop_fpll = 0.1;
    p.DipPct = config.DipPct;
    p.Vpeak_g1_init = p.Vpeak_n2;
    p.Vpeak_g1_fnl = p.Vpeak_n2 * (1 - config.DipPct/100);
    p.angle_g1_init = 0; p.angle_g1_fnl = 0;
    p.SCR = NaN; p.RX = NaN;
end

function p = apply_grid(p, SCR, RX)
    Zbase = p.Un1^2 / p.Sn_n2;
    Zgrid = Zbase / SCR;
    Xgrid = Zgrid / sqrt(1 + RX^2);
    p.Rac = RX * Xgrid;
    p.Lac = Xgrid / (2*pi*p.f1);
    p.SCR = SCR; p.RX = RX;
end

function p = compute_gains(p)
    omega_pll_n2 = 4 / (p.ts_pll_n2 * p.xi_pll_n2);
    p.kp_pll_n2 = p.xi_pll_n2 * 2 * omega_pll_n2 / p.Vpeak_n2;
    tau_pll_n2 = 2 * p.xi_pll_n2 / omega_pll_n2;
    p.ki_pll_n2 = p.kp_pll_n2 / tau_pll_n2;
    p.kp_s_n2 = p.Lc_n2 / p.taus_n2;
    p.ki_s_n2 = p.Rc_n2 / p.taus_n2;
    p.kp_p_n2 = p.taus_n2 / p.tau_p;
    p.ki_p_n2 = 1 / p.tau_p;
    p.kp_q_n2 = p.taus_n2 / p.tau_q;
    p.ki_q_n2 = 1 / p.tau_q;
end

function [simOut, ok] = run_sim(modelName, p)
    ok = false; simOut = struct();
    try
        fn = fieldnames(p);
        for i = 1:numel(fn)
            assignin('base', fn{i}, p.(fn{i}));
        end
        assignin('base', 'p', p);
        load_system(modelName);
        simOut = sim(modelName, 'StopTime', num2str(p.Tsim), ...
            'SaveOutput', 'on', 'ReturnWorkspaceOutputs', 'on');
        ok = true;
    catch
        ok = false;
    end
end

function [score, metrics, details] = stability_assessment(simOut, p, config)
    metrics = struct('isStable', false, 'score', 0, ...
        'H1_current_ok', false, 'H2_freq_bounds_ok', false, 'H3_freq_recovery_ok', false, ...
        'H4_damping_ok', false, 'H5_settling_ok', false, ...
        'I_peak_pu', NaN, 'I_final_pu', NaN, 'I_ratio', NaN, ...
        'f_max_dev_Hz', NaN, 'f_final_dev_Hz', NaN, 'zeta', NaN, 'T_settle', NaN, ...
        'S1_current', 0, 'S2_frequency', 0, 'S3_damping', 0, 'S4_settling', 0);
    details = struct();
    score = 0;

    try
        logsout = simOut.logsout;
    catch
        return;
    end

    f_nom = 50;
    I_nom = p.Inom_n2;
    t_start = p.T_stp + 0.02;

    % Current
    try
        sigI = logsout.get('is_abc_n1');
        t_i = sigI.Values.Time;
        iabc = squeeze(sigI.Values.Data);
        if size(iabc,1) == 3 && size(iabc,2) ~= 3, iabc = iabc'; end
        irms = sqrt(sum(iabc.^2,2)/3);
    catch
        return;
    end

    idx_i = t_i >= t_start;
    if nnz(idx_i) < 100, return; end
    t_post = t_i(idx_i);
    irms_post = irms(idx_i);

    N_avg = max(5, round(2*(1/f_nom)/mean(diff(t_i))));
    irms_smooth = movmean(irms_post, N_avg);

    I_peak = max(irms_smooth);
    I_final = mean(irms_smooth(end-max(50,round(0.1*length(irms_smooth)))+1:end));

    metrics.I_peak_pu = I_peak / I_nom;
    metrics.I_final_pu = I_final / I_nom;
    metrics.I_ratio = I_peak / max(I_final, 1);
    metrics.H1_current_ok = metrics.I_peak_pu <= config.criteria.I_limit_pu;

    % Frequency
    try
        sigF = logsout.get('omega');
        t_f = sigF.Values.Time;
        f = squeeze(sigF.Values.Data)/(2*pi);
    catch
        return;
    end

    idx_f = t_f >= t_start;
    if nnz(idx_f) < 50, return; end
    f_post = f(idx_f);
    if any(~isfinite(f_post)), return; end

    metrics.f_max_dev_Hz = max(abs(f_post - f_nom));
    metrics.f_final_dev_Hz = abs(mean(f_post(end-max(20,round(0.1*length(f_post)))+1:end)) - f_nom);
    metrics.H2_freq_bounds_ok = metrics.f_max_dev_Hz <= config.criteria.f_bound_Hz;
    metrics.H3_freq_recovery_ok = metrics.f_final_dev_Hz <= config.criteria.f_recovery_Hz;

    % Damping (conservative: minimum of log decrement and half-window amplitude decay)
    [peaks, ~] = findpeaks(irms_smooth);
    zeta1 = NaN;
    if numel(peaks) >= 3
        deltas = [];
        for i = 1:min(5,numel(peaks)-1)
            if peaks(i) > peaks(i+1) && peaks(i+1) > 0
                deltas(end+1) = log(peaks(i)/peaks(i+1)); %#ok<AGROW>
            end
        end
        if ~isempty(deltas)
            d = mean(deltas);
            zeta1 = d / sqrt(4*pi^2 + d^2);
        end
    end

    N_half = round(length(irms_smooth)/2);
    A1 = std(irms_smooth(1:N_half));
    A2 = std(irms_smooth(N_half+1:end));
    if A1 > 1e-9 && A2 < A1
        n_cycles = ((t_post(end)-t_post(1))/2) * f_nom;
        zeta2 = -log(max(A2/A1, eps)) / (pi * max(n_cycles, eps));
    elseif A2 > 1.05*A1
        zeta2 = -0.01;
    else
        zeta2 = 0.3;
    end

    if isfinite(zeta1)
        metrics.zeta = min(zeta1, zeta2);
    else
        metrics.zeta = zeta2;
    end
    metrics.H4_damping_ok = metrics.zeta >= config.criteria.zeta_min;

    % Settling
    band = 0.05 * I_final;
    in_band = abs(irms_smooth - I_final) <= band;
    k_last_out = find(~in_band, 1, 'last');
    if isempty(k_last_out)
        metrics.T_settle = 0;
    elseif k_last_out < numel(t_post)
        metrics.T_settle = t_post(k_last_out+1) - t_start;
    else
        metrics.T_settle = config.criteria.T_settle_max + 1;
    end
    metrics.H5_settling_ok = metrics.T_settle <= config.criteria.T_settle_max;

    metrics.isStable = metrics.H1_current_ok && metrics.H2_freq_bounds_ok && ...
                       metrics.H3_freq_recovery_ok && metrics.H4_damping_ok && metrics.H5_settling_ok;

    if metrics.isStable
        metrics.S1_current = max(0, min(1, (1.5 - metrics.I_ratio)/0.5));
        metrics.S2_frequency = max(0, min(1, 1 - metrics.f_max_dev_Hz/config.criteria.f_bound_Hz));
        metrics.S3_damping = max(0, min(1, (metrics.zeta - config.criteria.zeta_min)/(0.15 - config.criteria.zeta_min)));
        metrics.S4_settling = max(0, min(1, 1 - metrics.T_settle/config.criteria.T_settle_max));
        score = config.weights.current*metrics.S1_current + ...
                config.weights.frequency*metrics.S2_frequency + ...
                config.weights.damping*metrics.S3_damping + ...
                config.weights.settling*metrics.S4_settling;
    end

    metrics.score = score;
    details.t_post = t_post;
    details.irms_smooth = irms_smooth;
    details.f_post = f_post;
end
