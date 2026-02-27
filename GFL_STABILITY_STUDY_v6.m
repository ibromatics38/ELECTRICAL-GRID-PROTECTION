%% GFL_STABILITY_STUDY_v6.m
% =========================================================================
% AI-ASSISTED GFL INVERTER STABILITY ANALYSIS - VERSION 6 (FAIR + ROBUST)
% =========================================================================
% Major upgrades over v5:
%   1) Strict fair comparison: SAME candidate set + SAME evaluation cases
%   2) Stability criterion formalized with explicit windows and normalization
%   3) Additional hard metric: RoCoF limit (optional toggle)
%   4) Robust damping estimation from envelope decay (frequency and current)
%   5) More diagnostic plots (10 publication-oriented figures)
%   6) Reproducible output tables (CSV + MAT)
%
% Author: LAWAL IBRAHIM OKIKIOLA
% Updated by: AI assistant
% =========================================================================

clear; clc; close all;
rng(42, 'twister');

%% ============================== CONFIG ==================================
config.modelName = 'GFL_LCL_WeakGrid_AI';
config.T_end = 5.0;
config.T_stp = 1.3;
config.output_dir = fullfile(pwd, 'results_v6');
if ~exist(config.output_dir, 'dir'), mkdir(config.output_dir); end

% Grid set: SAME set used for data generation, fitting, and final comparison
config.SCR_list = [0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 2, 2.5, 3, 4, 5, 7, 10];
config.RX_list  = [0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 3.0];

% Common hyper-grid of candidate controllers (shared by all strategies)
config.taus_sweep = [0.3, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0, 5.0, 8.0] * 1e-3;   % s
config.tspll_sweep = [0.012, 0.018, 0.025, 0.035, 0.050, 0.080, 0.120, 0.160, 0.200]; % s

% Bandwidth rule
config.bw_separation = 0.20;  % omega_pll <= 0.20*omega_ci

% Formalized hard criteria
config.criteria.I_peak_limit_pu = 1.15;    % H1
config.criteria.f_transient_limit_Hz = 1.0; % H2
config.criteria.f_final_limit_Hz = 0.05;    % H3
config.criteria.zeta_min = 0.02;            % H4
config.criteria.T_settle_max_s = 2.0;       % H5
config.criteria.use_rocof = true;           % H6 optional
config.criteria.rocof_limit_Hzps = 2.0;

% Explicit analysis windows (critical for reproducibility)
config.window.post_step_delay_s = 0.02;
config.window.final_avg_s = 0.10;
config.window.settle_tol_pct = 0.02;        % ±2% of nominal frequency
config.window.smoothing_cycles = 2;

% Score weights
config.weights = struct('current',0.20,'freqTransient',0.25,'freqFinal',0.20,'damping',0.20,'settling',0.15);

%% ============================== HEADER ==================================
fprintf('\n=== GFL Stability Study v6 ===\n');
fprintf('Grid cases: %d x %d = %d\n', numel(config.SCR_list), numel(config.RX_list), numel(config.SCR_list)*numel(config.RX_list));
fprintf('Strategies: Baseline, LinearPLL, LinearBoth, LookupTable, GPR_AI\n');
fprintf('Fairness rule: all methods trained/evaluated from identical case pool\n\n');

%% ====================== PHASE 1: BEST-PER-CASE DATASET ==================
[training_data, candidate_bank] = build_oracle_dataset(config);

trainable_idx = find([training_data.has_stable]);
X_train = [[training_data(trainable_idx).SCR]', [training_data(trainable_idx).RX]'];
Y_taus = [training_data(trainable_idx).best_taus]';
Y_tspll = [training_data(trainable_idx).best_tspll]';
Y_score = [training_data(trainable_idx).best_score]';

fprintf('Trainable cases: %d/%d\n', numel(trainable_idx), numel(training_data));

%% ====================== PHASE 2: FIT ALL STRATEGIES =====================
methods = fit_strategies_v6(X_train, Y_taus, Y_tspll);
strategies = {'Baseline','LinearPLL','LinearBoth','LookupTable','GPR_AI'};

%% ====================== PHASE 3: GLOBAL FAIR EVALUATION =================
results = evaluate_all_cases(config, methods, strategies);
summary = summarize_results(results, strategies);

%% ====================== PHASE 4: EXPORT TABLES ==========================
T = struct2table(results);
writetable(T, fullfile(config.output_dir, 'results_casewise_v6.csv'));
summary_tbl = summary_to_table(summary, strategies);
writetable(summary_tbl, fullfile(config.output_dir, 'results_summary_v6.csv'));

%% ====================== PHASE 5: FIGURES (10) ===========================
generate_figures_v6(results, summary, strategies, config, training_data, Y_score, candidate_bank);

%% ====================== PHASE 6: SAVE ===================================
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
mat_path = fullfile(config.output_dir, sprintf('GFL_study_v6_%s.mat', timestamp));
save(mat_path, 'results', 'summary', 'methods', 'training_data', 'config', 'candidate_bank');

fprintf('\nSaved:\n  %s\n', mat_path);
fprintf('  %s\n', config.output_dir);

%% ============================ FUNCTIONS ==================================

function [training_data, candidate_bank] = build_oracle_dataset(config)
n_SCR = numel(config.SCR_list); n_RX = numel(config.RX_list);
n_cases = n_SCR*n_RX;
training_data = repmat(struct('SCR',NaN,'RX',NaN,'best_taus',NaN,'best_tspll',NaN,'best_score',-Inf,'has_stable',false,'n_stable',0), n_cases, 1);
candidate_bank = cell(n_cases,1);

k = 0;
for i=1:n_SCR
    for j=1:n_RX
        k = k + 1;
        SCR = config.SCR_list(i); RX = config.RX_list(j);
        p = get_base_params(config, SCR, RX);

        best = struct('score',-Inf,'taus',NaN,'tspll',NaN,'n_stable',0);
        local_candidates = [];

        for taus = config.taus_sweep
            for tspll = config.tspll_sweep
                if ~passes_bw_rule(taus, tspll, config), continue; end
                [metrics, ok] = run_simulation(config, p, taus, tspll);
                if ~ok, continue; end

                [is_stable, score, det] = evaluate_stability_v6(metrics, config);
                rec = struct('taus',taus,'tspll',tspll,'stable',is_stable,'score',score,'Ipk',det.I_peak_pu,'fmax',det.f_max_dev,'ffinal',det.f_final_dev,'zeta',det.zeta,'tset',det.T_settle,'rocof',det.rocof_max);
                local_candidates = [local_candidates; rec]; %#ok<AGROW>

                if is_stable
                    best.n_stable = best.n_stable + 1;
                    if score > best.score
                        best.score = score; best.taus = taus; best.tspll = tspll;
                    end
                end
            end
        end

        training_data(k).SCR = SCR;
        training_data(k).RX = RX;
        training_data(k).best_taus = best.taus;
        training_data(k).best_tspll = best.tspll;
        training_data(k).best_score = best.score;
        training_data(k).has_stable = isfinite(best.taus);
        training_data(k).n_stable = best.n_stable;
        candidate_bank{k} = local_candidates;

        if training_data(k).has_stable
            fprintf('[%3d/%3d] SCR=%.2f RX=%.2f  stable=%d  best=%.3f\n',k,n_cases,SCR,RX,best.n_stable,best.score);
        else
            fprintf('[%3d/%3d] SCR=%.2f RX=%.2f  NO STABLE CANDIDATE\n',k,n_cases,SCR,RX);
        end
    end
end
end

function ok = passes_bw_rule(taus, tspll, config)
omega_ci = 1/taus;
omega_pll = 4/(tspll*0.707);
ok = omega_pll <= config.bw_separation*omega_ci;
end

function methods = fit_strategies_v6(X_train, Y_taus, Y_tspll)
SCR = X_train(:,1); RX = X_train(:,2);

methods.baseline_taus = mean(Y_taus);
methods.baseline_tspll = mean(Y_tspll);

methods.linpll_taus = methods.baseline_taus;
methods.linpll_coef = polyfit(1./SCR, Y_tspll, 1);

methods.linboth_taus_coef = polyfit([1./SCR RX], Y_taus, 1);   % affine in [1/SCR, RX]
methods.linboth_tspll_coef = polyfit([1./SCR RX], Y_tspll, 1);

methods.SCR_zones = [0,1.5,3.0,6.0,Inf];
methods.RX_zones = [0,0.3,1.0,Inf];
methods.lut_taus = zone_avg(X_train, Y_taus, methods.SCR_zones, methods.RX_zones, methods.baseline_taus);
methods.lut_tspll = zone_avg(X_train, Y_tspll, methods.SCR_zones, methods.RX_zones, methods.baseline_tspll);

Xg = [X_train, log10(SCR), log10(RX), SCR.*RX, 1./SCR, 1./(RX+0.05)];
methods.gpr_taus = fitrgp(Xg, Y_taus, 'KernelFunction','ardsquaredexponential','Standardize',true,'Sigma',0.001);
methods.gpr_tspll = fitrgp(Xg, Y_tspll, 'KernelFunction','ardsquaredexponential','Standardize',true,'Sigma',0.001);
end

function coef = polyfit(X, y, ~)
Xaug = [ones(size(X,1),1), X];
coef = Xaug\y;
end

function y = polyval(coef, X)
X = X(:);
if numel(coef)==2
    y = coef(1) + coef(2)*X;
else
    if size(X,2)==1
        X = [X zeros(size(X))];
    end
    y = coef(1) + X*coef(2:end);
end
end

function lut = zone_avg(X, Y, scrz, rxz, fallback)
lut = zeros(numel(scrz)-1, numel(rxz)-1);
for i=1:size(lut,1)
    for j=1:size(lut,2)
        idx = X(:,1)>scrz(i) & X(:,1)<=scrz(i+1) & X(:,2)>rxz(j) & X(:,2)<=rxz(j+1);
        if any(idx), lut(i,j)=mean(Y(idx)); else, lut(i,j)=fallback; end
    end
end
end

function results = evaluate_all_cases(config, methods, strategies)
n_cases = numel(config.SCR_list)*numel(config.RX_list);
results = repmat(struct(), n_cases, 1);
k=0;
for SCR = config.SCR_list
    for RX = config.RX_list
        k = k + 1;
        p = get_base_params(config, SCR, RX);
        results(k).SCR = SCR; results(k).RX = RX;

        for s = 1:numel(strategies)
            st = strategies{s};
            [taus, tspll] = get_method_params(methods, st, SCR, RX);
            results(k).(['taus_' st]) = taus;
            results(k).(['tspll_' st]) = tspll;

            [metrics, ok] = run_simulation(config, p, taus, tspll);
            if ~ok
                [results(k).(['stable_' st]),results(k).(['score_' st])] = deal(false,0);
                [results(k).(['Ipk_' st]),results(k).(['fmax_' st]),results(k).(['ffinal_' st]),results(k).(['zeta_' st]),results(k).(['tset_' st]),results(k).(['rocof_' st])] = deal(NaN);
                continue;
            end
            [stable, score, det] = evaluate_stability_v6(metrics, config);
            results(k).(['stable_' st]) = stable;
            results(k).(['score_' st]) = score;
            results(k).(['Ipk_' st]) = det.I_peak_pu;
            results(k).(['fmax_' st]) = det.f_max_dev;
            results(k).(['ffinal_' st]) = det.f_final_dev;
            results(k).(['zeta_' st]) = det.zeta;
            results(k).(['tset_' st]) = det.T_settle;
            results(k).(['rocof_' st]) = det.rocof_max;
        end
    end
end
end

function summary = summarize_results(results, strategies)
n = numel(results);
for s = 1:numel(strategies)
    st = strategies{s};
    stable = [results.(['stable_' st])];
    score = [results.(['score_' st])];
    zeta = [results.(['zeta_' st])];
    summary.(st).n_stable = sum(stable);
    summary.(st).rate = 100*mean(stable);
    summary.(st).mean_score_stable = mean(score(stable));
    summary.(st).mean_score_all = mean(score);
    summary.(st).mean_zeta_stable = mean(zeta(stable));
    summary.(st).n_total = n;
    if isnan(summary.(st).mean_score_stable), summary.(st).mean_score_stable = 0; end
    if isnan(summary.(st).mean_zeta_stable), summary.(st).mean_zeta_stable = 0; end
end
end

function tbl = summary_to_table(summary, strategies)
rows = cell(numel(strategies), 6);
for i=1:numel(strategies)
    st = strategies{i};
    rows{i,1}=st;
    rows{i,2}=summary.(st).n_stable;
    rows{i,3}=summary.(st).n_total;
    rows{i,4}=summary.(st).rate;
    rows{i,5}=summary.(st).mean_score_stable;
    rows{i,6}=summary.(st).mean_zeta_stable;
end
tbl = cell2table(rows, 'VariableNames', {'Strategy','StableCases','TotalCases','StabilityRatePct','MeanScoreStable','MeanZetaStable'});
end

function params = get_base_params(config, SCR, RX)
params.Vbase = 400; params.Sbase = 5000; params.fbase = 50; params.Vdc = 750;
Zbase = params.Vbase^2 / params.Sbase;
Zg = Zbase / SCR;
theta = atan(1/RX);
params.Rg = Zg*cos(theta);
params.Lg = Zg*sin(theta)/(2*pi*params.fbase);
params.Lc = 2.5e-3; params.Rc = 0.1; params.Lf = 0.5e-3; params.Rf = 0.05; params.Cf = 10e-6;
params.T_stp = config.T_stp; params.T_end = config.T_end;
end

function [taus, tspll] = get_method_params(methods, strategy, SCR, RX)
switch strategy
    case 'Baseline'
        taus = methods.baseline_taus; tspll = methods.baseline_tspll;
    case 'LinearPLL'
        taus = methods.linpll_taus;
        tspll = polyval(methods.linpll_coef, 1/SCR);
    case 'LinearBoth'
        taus = polyval(methods.linboth_taus_coef, [1/SCR RX]);
        tspll = polyval(methods.linboth_tspll_coef, [1/SCR RX]);
    case 'LookupTable'
        iz = find(SCR > methods.SCR_zones(1:end-1) & SCR <= methods.SCR_zones(2:end), 1, 'first');
        jz = find(RX > methods.RX_zones(1:end-1) & RX <= methods.RX_zones(2:end), 1, 'first');
        if isempty(iz), iz = size(methods.lut_taus,1); end
        if isempty(jz), jz = size(methods.lut_taus,2); end
        taus = methods.lut_taus(iz,jz); tspll = methods.lut_tspll(iz,jz);
    case 'GPR_AI'
        X = [SCR, RX, log10(SCR), log10(RX), SCR*RX, 1/SCR, 1/(RX+0.05)];
        taus = predict(methods.gpr_taus, X); tspll = predict(methods.gpr_tspll, X);
    otherwise
        error('Unknown strategy');
end
% Physical clamps
taus = max(0.3e-3, min(8e-3, taus));
tspll = max(0.012, min(0.200, tspll));
end

function [metrics, success] = run_simulation(config, p, taus, tspll)
metrics = struct(); success = false;
try
    xi = 0.707;
    omega_pll = 4/(tspll*xi);
    Vpeak = p.Vbase*sqrt(2/3);
    kp_pll = xi*2*omega_pll/Vpeak;
    ki_pll = kp_pll/0.1;
    kp_s = p.Lc/taus; ki_s = p.Rc/taus;

    assignin('base','Vdc',p.Vdc); assignin('base','Rg',p.Rg); assignin('base','Lg',p.Lg);
    assignin('base','Lc',p.Lc); assignin('base','Rc',p.Rc); assignin('base','Lf',p.Lf); assignin('base','Rf',p.Rf); assignin('base','Cf',p.Cf);
    assignin('base','kp_pll',kp_pll); assignin('base','ki_pll',ki_pll); assignin('base','kp_s',kp_s); assignin('base','ki_s',ki_s);
    assignin('base','T_stp',p.T_stp); assignin('base','T_end',p.T_end);

    load_system(config.modelName);
    simOut = sim(config.modelName, 'StopTime', num2str(p.T_end));

    t = simOut.tout;
    try
        iabc = simOut.logsout.getElement('is_abc_n1').Values.Data;
    catch
        iabc = simOut.logsout.getElement('is_abc').Values.Data;
    end
    omega = simOut.logsout.getElement('omega').Values.Data;

    I_rms = sqrt(sum(iabc.^2,2)/3);
    f_hz = omega(:)/(2*pi);

    metrics.t = t(:);
    metrics.I_rms = I_rms(:);
    metrics.f_hz = f_hz(:);
    metrics.I_base = p.Sbase/(sqrt(3)*p.Vbase);
    metrics.f_nom = 50;
    metrics.t_step = p.T_stp;
    success = true;
catch
    success = false;
end
end

function [is_stable, score, details] = evaluate_stability_v6(metrics, config)
t = metrics.t; I = metrics.I_rms; f = metrics.f_hz;
f_nom = metrics.f_nom;

post_start = metrics.t_step + config.window.post_step_delay_s;
idx_post = t >= post_start;
if nnz(idx_post) < 20
    [is_stable, score, details] = fail_packet(); return;
end

tp = t(idx_post); Ip = I(idx_post); fp = f(idx_post);

% smoothing
Ts = median(diff(tp));
Nsm = max(3, round(config.window.smoothing_cycles*(1/f_nom)/Ts));
Ip_sm = movmean(Ip, Nsm);
fp_sm = movmean(fp, Nsm);

% H1 current peak (p.u.)
I_peak_pu = max(Ip_sm)/metrics.I_base;

% H2 transient frequency deviation
f_max_dev = max(abs(fp_sm - f_nom));

% H3 final frequency (last fixed window)
idx_final = tp >= (tp(end)-config.window.final_avg_s);
f_final_dev = abs(mean(fp_sm(idx_final)) - f_nom);

% H4 damping (conservative: min(current envelope damping, frequency envelope damping))
zeta_I = estimate_damping(tp, Ip_sm-mean(Ip_sm));
zeta_f = estimate_damping(tp, fp_sm-f_nom);
zeta = min(zeta_I, zeta_f);

% H5 settling time on frequency (strict: remain in band after enter)
tol = config.window.settle_tol_pct*abs(f_nom);
in_band = abs(fp_sm-f_nom) <= tol;
last_out = find(~in_band, 1, 'last');
if isempty(last_out), T_settle = 0; else, T_settle = tp(last_out)-tp(1); end

% H6 RoCoF (optional)
rocof = [0; abs(diff(fp_sm)./max(diff(tp), eps))];
rocof_max = max(rocof);

H1 = I_peak_pu <= config.criteria.I_peak_limit_pu;
H2 = f_max_dev <= config.criteria.f_transient_limit_Hz;
H3 = f_final_dev <= config.criteria.f_final_limit_Hz;
H4 = zeta >= config.criteria.zeta_min;
H5 = T_settle <= config.criteria.T_settle_max_s;
if config.criteria.use_rocof
    H6 = rocof_max <= config.criteria.rocof_limit_Hzps;
else
    H6 = true;
end

is_stable = H1 && H2 && H3 && H4 && H5 && H6;

S1 = max(0, 1 - (I_peak_pu-1)/0.4);
S2 = max(0, 1 - f_max_dev/config.criteria.f_transient_limit_Hz);
S3 = max(0, 1 - f_final_dev/config.criteria.f_final_limit_Hz);
S4 = max(0, min(1, (zeta - config.criteria.zeta_min)/(0.15 - config.criteria.zeta_min)));
S5 = max(0, 1 - T_settle/config.criteria.T_settle_max_s);

score = is_stable * (config.weights.current*S1 + config.weights.freqTransient*S2 + config.weights.freqFinal*S3 + config.weights.damping*S4 + config.weights.settling*S5);
score = max(0, min(1, score));

details = struct('I_peak_pu',I_peak_pu,'f_max_dev',f_max_dev,'f_final_dev',f_final_dev,'zeta',zeta,'T_settle',T_settle,'rocof_max',rocof_max,'H1',H1,'H2',H2,'H3',H3,'H4',H4,'H5',H5,'H6',H6);
end

function zeta = estimate_damping(t, x)
x = x(:); t = t(:);
if numel(x) < 20 || std(x) < 1e-6
    zeta = 0.3; return;
end
[pk,~] = findpeaks(abs(x));
if numel(pk) >= 3
    n = min(5, numel(pk));
    pk = pk(1:n);
    deltas = log(pk(1:end-1)./max(pk(2:end), eps));
    d = mean(deltas(deltas>0));
    if isempty(d) || ~isfinite(d), zeta = 0.01; return; end
    zeta = d / sqrt(4*pi^2 + d^2);
else
    half = floor(numel(x)/2);
    a1 = std(x(1:half)); a2 = std(x(half+1:end));
    if a2 <= a1, zeta = 0.05; else, zeta = -0.01; end
end
zeta = max(-0.1, min(1.0, zeta));
end

function [is_stable, score, details] = fail_packet()
is_stable = false; score = 0;
details = struct('I_peak_pu',NaN,'f_max_dev',NaN,'f_final_dev',NaN,'zeta',NaN,'T_settle',NaN,'rocof_max',NaN,'H1',false,'H2',false,'H3',false,'H4',false,'H5',false,'H6',false);
end

function generate_figures_v6(results, summary, strategies, config, training_data, Y_score, candidate_bank)
% Fig1: stability rate
rates = cellfun(@(s) summary.(s).rate, strategies);
figure('Position',[100 100 900 450]); bar(rates); grid on; ylim([0 110]);
set(gca,'XTickLabel',strategies); ylabel('Stability Rate (%)'); title('Fig1 Stability Rate');
saveas(gcf, fullfile(config.output_dir,'fig1_stability_rate.png'));

% Fig2: mean stable score
means = cellfun(@(s) summary.(s).mean_score_stable, strategies);
figure('Position',[100 100 900 450]); bar(means); grid on; ylim([0 1]);
set(gca,'XTickLabel',strategies); ylabel('Mean Score (stable only)'); title('Fig2 Mean Score');
saveas(gcf, fullfile(config.output_dir,'fig2_mean_score.png'));

% Fig3: score heatmaps per strategy
figure('Position',[50 50 1500 300]);
for s=1:numel(strategies)
    st = strategies{s};
    M = matrix_from_results(results, config, ['score_' st]);
    subplot(1,numel(strategies),s); imagesc(config.SCR_list, config.RX_list, M); set(gca,'YDir','normal'); caxis([0 1]); colorbar;
    xlabel('SCR'); if s==1, ylabel('R/X'); end; title(st);
end
sgtitle('Fig3 Score Heatmaps'); saveas(gcf, fullfile(config.output_dir,'fig3_heatmaps_score.png'));

% Fig4: stable masks
figure('Position',[50 50 1500 300]);
for s=1:numel(strategies)
    st = strategies{s};
    M = matrix_from_results(results, config, ['stable_' st]);
    subplot(1,numel(strategies),s); imagesc(config.SCR_list, config.RX_list, M); set(gca,'YDir','normal'); caxis([0 1]); colorbar;
    xlabel('SCR'); if s==1, ylabel('R/X'); end; title(st);
end
sgtitle('Fig4 Stability Masks'); saveas(gcf, fullfile(config.output_dir,'fig4_heatmaps_stable.png'));

% Fig5: criterion margins for GPR
st = 'GPR_AI';
figure('Position',[100 100 1000 600]);
subplot(2,2,1); scatter([results.SCR],[results.(['Ipk_' st])],30,[results.RX],'filled'); colorbar; xlabel('SCR'); ylabel('I_{peak} p.u.'); yline(config.criteria.I_peak_limit_pu,'r--'); title('Fig5a H1 margin');
subplot(2,2,2); scatter([results.SCR],[results.(['fmax_' st])],30,[results.RX],'filled'); colorbar; xlabel('SCR'); ylabel('\Deltaf_{max} (Hz)'); yline(config.criteria.f_transient_limit_Hz,'r--'); title('Fig5b H2 margin');
subplot(2,2,3); scatter([results.SCR],[results.(['ffinal_' st])],30,[results.RX],'filled'); colorbar; xlabel('SCR'); ylabel('\Deltaf_{final} (Hz)'); yline(config.criteria.f_final_limit_Hz,'r--'); title('Fig5c H3 margin');
subplot(2,2,4); scatter([results.SCR],[results.(['zeta_' st])],30,[results.RX],'filled'); colorbar; xlabel('SCR'); ylabel('\zeta'); yline(config.criteria.zeta_min,'r--'); title('Fig5d H4 margin');
saveas(gcf, fullfile(config.output_dir,'fig5_gpr_criteria_margins.png'));

% Fig6: parameter surfaces for strategies
figure('Position',[100 100 1300 500]);
for s=1:numel(strategies)
    st = strategies{s};
    subplot(2,numel(strategies),s);
    scatter3([results.SCR],[results.RX],1000*[results.(['taus_' st])],25,[results.(['score_' st])],'filled'); grid on; xlabel('SCR'); ylabel('R/X'); zlabel('\tau_s (ms)'); title([st ' taus']);
    subplot(2,numel(strategies),s+numel(strategies));
    scatter3([results.SCR],[results.RX],1000*[results.(['tspll_' st])],25,[results.(['score_' st])],'filled'); grid on; xlabel('SCR'); ylabel('R/X'); zlabel('t_{s,pll} (ms)'); title([st ' tspll']);
end
saveas(gcf, fullfile(config.output_dir,'fig6_parameter_surfaces.png'));

% Fig7: distribution of oracle stable candidate count
figure('Position',[100 100 800 450]);
histogram([training_data.n_stable]); grid on; xlabel('# stable candidates per case'); ylabel('count'); title('Fig7 Oracle candidate richness');
saveas(gcf, fullfile(config.output_dir,'fig7_oracle_stable_count_hist.png'));

% Fig8: oracle best score map
Mbest = matrix_from_training(training_data, config, 'best_score');
figure('Position',[100 100 800 450]); imagesc(config.SCR_list, config.RX_list, Mbest); set(gca,'YDir','normal'); colorbar; caxis([0 1]);
xlabel('SCR'); ylabel('R/X'); title('Fig8 Oracle best achievable score map');
saveas(gcf, fullfile(config.output_dir,'fig8_oracle_best_score_map.png'));

% Fig9: fairness check - same candidate count per case
cand_counts = cellfun(@numel, candidate_bank);
figure('Position',[100 100 800 450]);
plot(cand_counts,'o-'); grid on; xlabel('Case index'); ylabel('Evaluated candidates');
title('Fig9 Fairness check (candidate bank size per case)');
saveas(gcf, fullfile(config.output_dir,'fig9_fairness_candidate_count.png'));

% Fig10: train target distributions
figure('Position',[100 100 1000 350]);
subplot(1,3,1); histogram(1e3*[training_data.best_taus]); xlabel('best taus (ms)'); grid on;
subplot(1,3,2); histogram(1e3*[training_data.best_tspll]); xlabel('best tspll (ms)'); grid on;
subplot(1,3,3); histogram(Y_score); xlabel('oracle best score'); grid on;
sgtitle('Fig10 Training target distributions');
saveas(gcf, fullfile(config.output_dir,'fig10_training_distributions.png'));
end

function M = matrix_from_results(results, config, field)
M = nan(numel(config.RX_list), numel(config.SCR_list));
for k=1:numel(results)
    i = find(abs(config.SCR_list-results(k).SCR)<1e-12,1);
    j = find(abs(config.RX_list-results(k).RX)<1e-12,1);
    M(j,i) = double(results(k).(field));
end
end

function M = matrix_from_training(training_data, config, field)
M = nan(numel(config.RX_list), numel(config.SCR_list));
for k=1:numel(training_data)
    i = find(abs(config.SCR_list-training_data(k).SCR)<1e-12,1);
    j = find(abs(config.RX_list-training_data(k).RX)<1e-12,1);
    M(j,i) = training_data(k).(field);
end
end
