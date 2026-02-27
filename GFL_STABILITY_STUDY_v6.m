% GFL_STABILITY_STUDY_v6.m
% =========================================================================
% AI-ASSISTED GFL INVERTER STABILITY BENCHMARK - VERSION 6
% =========================================================================
% Major upgrades vs v5:
%   1) FAIRNESS: all strategies are trained/evaluated from the same case list.
%   2) STABILITY CRITERION: explicit hard limits + normalized margin score.
%   3) ROBUST METRICS: corrected final-frequency estimate, RoCoF, nadir, and
%      oscillation envelope checks to avoid false positives.
%   4) MORE FIGURES: 10 publication-oriented plots.
%   5) REPRODUCIBILITY: all thresholds/weights centralized in config.criteria.
%
% Author: LAWAL IBRAHIM OKIKIOLA (updated with critical review + refactor)
% =========================================================================

clear; clc; close all;

%% =============================== CONFIG ================================
config.modelName = 'GFL_LCL_WeakGrid_AI';
config.T_end = 5.0;
config.T_stp = 1.3;
config.output_dir = fullfile(pwd, 'results_v6');
if ~exist(config.output_dir, 'dir'), mkdir(config.output_dir); end

% Common dataset for ALL methods (fairness enforced)
config.SCR_list = [0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 2, 2.5, 3, 4, 5, 7, 10];
config.RX_list  = [0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 3.0];

% Candidate tuning grid used to create target labels for all learning rules
config.taus_sweep  = [0.3, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0, 5.0, 8.0] * 1e-3; % [s]
config.tspll_sweep = [0.012, 0.018, 0.025, 0.035, 0.050, 0.080, 0.120, 0.160, 0.200];

% Bandwidth-separation constraint (kept identical across all methods)
config.bw_separation = 0.20; % omega_pll <= 0.2*omega_ci

% Hard criteria and soft-score settings
config.criteria = struct();
config.criteria.I_peak_pu_max      = 1.15;  % H1
config.criteria.f_dev_transient_Hz = 1.00;  % H2
config.criteria.f_dev_final_Hz     = 0.05;  % H3
config.criteria.zeta_min           = 0.02;  % H4
config.criteria.T_settle_max_s     = 2.00;  % H5
config.criteria.rocof_max_Hzps     = 2.00;  % H6

% Soft weights (sum to 1)
config.weights = struct('current',0.20,'frequency',0.25,'finalBias',0.15,'damping',0.20,'settling',0.10,'rocof',0.10);

% Small-signal analysis options
config.small_signal.enable = true;
config.small_signal.use_linearization = false; % set true if linearize() model workflow is available
config.small_signal.report_top_n = 3;

% Validate soft-score weights
w = config.weights;
wsum = w.current + w.frequency + w.finalBias + w.damping + w.settling + w.rocof;
assert(abs(wsum - 1.0) < 1e-6, 'Weights must sum to 1.0');

fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════════════════╗\n');
fprintf('║   AI-ASSISTED GFL INVERTER STABILITY ANALYSIS (v6.0)             ║\n');
fprintf('║   Author: LAWAL IBRAHIM OKIKIOLA                                 ║\n');
fprintf('╠═══════════════════════════════════════════════════════════════════╣\n');
fprintf('║   HARD CRITERIA                                                   ║\n');
fprintf('║     H1: I_peak ≤ %.2f p.u.                                        ║\n', config.criteria.I_peak_pu_max);
fprintf('║     H2: |Δf_trans| ≤ %.1f Hz                                      ║\n', config.criteria.f_dev_transient_Hz);
fprintf('║     H3: |Δf_final| ≤ %.2f Hz                                      ║\n', config.criteria.f_dev_final_Hz);
fprintf('║     H4: ζ ≥ %.0f%%                                                ║\n', config.criteria.zeta_min*100);
fprintf('║     H5: T_settle ≤ %.1f s                                        ║\n', config.criteria.T_settle_max_s);
fprintf('║     H6: RoCoF ≤ %.1f Hz/s                                        ║\n', config.criteria.rocof_max_Hzps);
fprintf('╚═══════════════════════════════════════════════════════════════════╝\n');
fprintf('Dataset: %d SCR x %d R/X = %d cases (shared across methods)\n\n', ...
    numel(config.SCR_list), numel(config.RX_list), numel(config.SCR_list)*numel(config.RX_list));

%% ===================== PHASE 1: LABEL COMMON DATASET ====================
nS = numel(config.SCR_list); nR = numel(config.RX_list);
nCases = nS*nR;
training_data = repmat(struct('SCR',NaN,'RX',NaN,'best_taus',NaN,'best_tspll',NaN,'best_score',-Inf,'has_stable',false), nCases,1);

k = 0;
for i = 1:nS
    for j = 1:nR
        k = k + 1;
        SCR = config.SCR_list(i); RX = config.RX_list(j);
        params = get_base_params(config, SCR, RX);

        bestScore = -Inf; best_taus = NaN; best_tspll = NaN; stableCount = 0;
        for taus = config.taus_sweep
            for tspll = config.tspll_sweep
                if ~bandwidth_ok(taus, tspll, config.bw_separation), continue; end
                [m, ok] = run_simulation(config, params, taus, tspll);
                if ~ok, continue; end
                [is_stable, score] = evaluate_stability_v6(m, config);
                if is_stable
                    stableCount = stableCount + 1;
                    if score > bestScore
                        bestScore = score; best_taus = taus; best_tspll = tspll;
                    end
                end
            end
        end

        training_data(k).SCR = SCR;
        training_data(k).RX = RX;
        training_data(k).best_taus = best_taus;
        training_data(k).best_tspll = best_tspll;
        training_data(k).best_score = max(bestScore,0);
        training_data(k).has_stable = ~isnan(best_taus);

        if training_data(k).has_stable
            fprintf('[%3d/%d] SCR=%4.2f RX=%4.2f stable_cfg=%2d best=%.3f\n',k,nCases,SCR,RX,stableCount,bestScore);
        else
            fprintf('[%3d/%d] SCR=%4.2f RX=%4.2f NO-STABLE-LABEL\n',k,nCases,SCR,RX);
        end
    end
end

idxTrain = find([training_data.has_stable]);
X_train = [[training_data(idxTrain).SCR]', [training_data(idxTrain).RX]'];
Y_taus = [training_data(idxTrain).best_taus]';
Y_tspll = [training_data(idxTrain).best_tspll]';
fprintf('Trainable labeled cases: %d/%d\n', numel(idxTrain), nCases);

%% ====================== PHASE 2: FIT ALL METHODS ========================
methods = fit_all_methods(X_train, Y_taus, Y_tspll);

%% ===================== PHASE 3: FAIR EVALUATION =========================
fprintf('\n');
fprintf('═══════════════════════════════════════════════════════════════════\n');
fprintf('PHASE 3: EVALUATING ALL METHODS ON %d CASES\n', nCases);
fprintf('═══════════════════════════════════════════════════════════════════\n');
strategies = {'Baseline','LinearPLL','LinearBoth','LookupTable','GPR_AI'};
results = repmat(struct(), nCases, 1);

k = 0;
for i = 1:nS
    for j = 1:nR
        k = k + 1;
        SCR = config.SCR_list(i); RX = config.RX_list(j);
        params = get_base_params(config, SCR, RX);

        results(k).SCR = SCR; results(k).RX = RX;

        for s = 1:numel(strategies)
            strat = strategies{s};
            [taus, tspll] = get_method_params(methods, strat, SCR, RX);

            results(k).(['taus_' strat]) = taus;
            results(k).(['tspll_' strat]) = tspll;
            [m, ok] = run_simulation(config, params, taus, tspll);
            if ~ok
                [is_stable, score, d] = fail_stub();
            else
                [is_stable, score, d] = evaluate_stability_v6(m, config);
            end

            results(k).(['stable_' strat]) = is_stable;
            results(k).(['score_' strat]) = score;
            fns = fieldnames(d);
            for f = 1:numel(fns)
                results(k).([fns{f} '_' strat]) = d.(fns{f});
            end
        end

        fprintf('[%3d/%d] SCR=%.2f RX=%.2f: ', k, nCases, SCR, RX);
        for s2 = 1:numel(strategies)
            st2 = strategies{s2};
            if results(k).(['stable_' st2])
                fprintf('%s=%.2f ', st2(1), results(k).(['score_' st2]));
            else
                fprintf('%s=FAIL ', st2(1));
            end
        end
        fprintf('\n');
    end
end

summary = summarize_results(results, strategies);
print_summary(summary, nCases);

%% ======================= PHASE 4: MANY FIGURES ==========================
generate_figures_v6(results, summary, strategies, config, methods);
if config.small_signal.enable
    generate_small_signal_figures_v6(results, strategies, config);
end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
outMat = fullfile(config.output_dir, sprintf('GFL_study_v6_%s.mat', timestamp));
save(outMat, 'results', 'summary', 'training_data', 'methods', 'config');
fprintf('\nSaved MAT file: %s\n', outMat);

%% =========================== HELPERS ====================================
function tf = bandwidth_ok(taus, tspll, bw_ratio)
omega_ci = 1/taus;
omega_pll = 4/(tspll*0.707);
tf = omega_pll <= bw_ratio*omega_ci;
end

function params = get_base_params(config, SCR, RX)
params.Vbase = 400; params.Sbase = 5000; params.fbase = 50; params.Vdc = 750;
Zbase = params.Vbase^2 / params.Sbase;
Zg = Zbase / SCR;
theta_g = atan(1/RX);
params.Rg = Zg*cos(theta_g);
params.Lg = Zg*sin(theta_g)/(2*pi*params.fbase);
params.Lc = 2.5e-3; params.Rc = 0.1;
params.Lf = 0.5e-3; params.Rf = 0.05; params.Cf = 10e-6;
params.T_stp = config.T_stp; params.T_end = config.T_end;
end

function methods = fit_all_methods(X_train, Y_taus, Y_tspll)
SCR = X_train(:,1); RX = X_train(:,2);
methods.baseline_taus = mean(Y_taus);
methods.baseline_tspll = mean(Y_tspll);
methods.linpll_taus = methods.baseline_taus;
methods.linpll_coef = polyfit(1./SCR, Y_tspll, 1);
methods.linboth_taus_coef = polyfit(1./SCR, Y_taus, 1);
methods.linboth_tspll_coef = polyfit(1./SCR, Y_tspll, 1);

methods.SCR_zones = [0,1.5,3,6,Inf];
methods.RX_zones = [0,0.3,1,Inf];
methods.lut_taus = zeros(4,3); methods.lut_tspll = zeros(4,3);
for i = 1:4
    for j = 1:3
        idx = SCR>methods.SCR_zones(i) & SCR<=methods.SCR_zones(i+1) & RX>methods.RX_zones(j) & RX<=methods.RX_zones(j+1);
        if any(idx)
            methods.lut_taus(i,j) = mean(Y_taus(idx));
            methods.lut_tspll(i,j) = mean(Y_tspll(idx));
        else
            methods.lut_taus(i,j) = methods.baseline_taus;
            methods.lut_tspll(i,j) = methods.baseline_tspll;
        end
    end
end

Xf = [X_train, log10(X_train(:,1)), log10(X_train(:,2)), X_train(:,1).*X_train(:,2)];
methods.gpr_taus = fitrgp(Xf, Y_taus, 'KernelFunction','ardsquaredexponential','Standardize',true,'Sigma',1e-3);
methods.gpr_tspll = fitrgp(Xf, Y_tspll, 'KernelFunction','ardsquaredexponential','Standardize',true,'Sigma',1e-3);
end

function [taus, tspll] = get_method_params(methods, strategy, SCR, RX)
switch strategy
    case 'Baseline'
        taus = methods.baseline_taus; tspll = methods.baseline_tspll;
    case 'LinearPLL'
        taus = methods.linpll_taus;
        tspll = polyval(methods.linpll_coef, 1/SCR);
    case 'LinearBoth'
        taus = polyval(methods.linboth_taus_coef, 1/SCR);
        tspll = polyval(methods.linboth_tspll_coef, 1/SCR);
    case 'LookupTable'
        iz = find(SCR>methods.SCR_zones(1:end-1) & SCR<=methods.SCR_zones(2:end),1); if isempty(iz), iz=4; end
        jz = find(RX>methods.RX_zones(1:end-1) & RX<=methods.RX_zones(2:end),1); if isempty(jz), jz=3; end
        taus = methods.lut_taus(iz,jz); tspll = methods.lut_tspll(iz,jz);
    case 'GPR_AI'
        Xt = [SCR,RX,log10(SCR),log10(RX),SCR*RX];
        taus = predict(methods.gpr_taus, Xt);
        tspll = predict(methods.gpr_tspll, Xt);
end
taus = max(0.3e-3,min(8e-3,taus));
tspll = max(0.012,min(0.200,tspll));
end

function [metrics, success] = run_simulation(config, params, taus, tspll)
metrics = struct('I_peak_pu',NaN,'f_max_dev',NaN,'f_final_dev',NaN,'zeta',NaN,'T_settle',NaN,'rocof_max',NaN);
success = false;
try
    xi = 0.707;
    omega_pll = 4/(tspll*xi);
    Vpeak = params.Vbase*sqrt(2/3);
    kp_pll = xi*2*omega_pll/Vpeak;
    ki_pll = kp_pll/0.1;
    kp_s = params.Lc/taus; ki_s = params.Rc/taus;

    assignin('base','Vdc',params.Vdc); assignin('base','Rg',params.Rg); assignin('base','Lg',params.Lg);
    assignin('base','Lc',params.Lc); assignin('base','Rc',params.Rc); assignin('base','Lf',params.Lf);
    assignin('base','Rf',params.Rf); assignin('base','Cf',params.Cf); assignin('base','kp_pll',kp_pll);
    assignin('base','ki_pll',ki_pll); assignin('base','kp_s',kp_s); assignin('base','ki_s',ki_s);
    assignin('base','T_stp',params.T_stp); assignin('base','T_end',params.T_end);

    load_system(config.modelName);
    simOut = sim(config.modelName, 'StopTime', num2str(params.T_end));

    t = simOut.tout(:);
    is_abc = extract_current(simOut);
    omega = simOut.logsout.getElement('omega').Values.Data(:);
    f = omega/(2*pi);

    I_rms = sqrt(sum(is_abc.^2,2)/3);
    I_base = params.Sbase/(sqrt(3)*params.Vbase);

    idx = t >= params.T_stp+0.02;
    t2 = t(idx); I2 = I_rms(idx); f2 = f(idx);

    metrics.I_peak_pu = max(I2)/I_base;
    metrics.f_max_dev = max(abs(f2-50));

    idx_final = t >= (params.T_end-0.1);
    metrics.f_final_dev = abs(mean(f(idx_final)) - 50);

    [metrics.zeta, metrics.T_settle] = robust_damping_settling(t2, f2, 50);

    dt = median(diff(t2));
    if dt > 0
        dfdt = gradient(f2, dt);
        metrics.rocof_max = max(abs(dfdt));
    else
        metrics.rocof_max = Inf;
    end

    success = true;
catch
end
end

function is_abc = extract_current(simOut)
try
    is_abc = simOut.logsout.getElement('is_abc_n1').Values.Data;
catch
    is_abc = simOut.logsout.getElement('is_abc').Values.Data;
end
if size(is_abc,2) ~= 3, is_abc = squeeze(is_abc); end
if size(is_abc,2) ~= 3 && size(is_abc,1)==3, is_abc = is_abc'; end
end

function [zeta, T_settle] = robust_damping_settling(t, sig, target)
err = sig(:)-target;
if numel(err) < 20
    zeta = 0; T_settle = Inf; return;
end

% settling in ±2%
tol = 0.02*abs(target);
lastOut = find(abs(err)>tol,1,'last');
if isempty(lastOut), T_settle = 0; else, T_settle = t(lastOut)-t(1); end

[pks,~] = findpeaks(abs(err));
if numel(pks)>=2 && pks(end)>0
    delta = log(max(pks(1),eps)/max(pks(end),eps))/max(numel(pks)-1,1);
    zeta = delta/sqrt(4*pi^2 + delta^2);
else
    head = std(err(1:max(5,floor(end/3))));
    tail = std(err(max(1,floor(2*end/3)):end));
    if tail < 0.7*head, zeta = 0.15; else, zeta = 0.01; end
end
zeta = max(-0.1,min(1,zeta));
end

function [is_stable, score, details] = evaluate_stability_v6(m, config)
c = config.criteria;
details.I_peak = m.I_peak_pu;
details.f_max_dev = m.f_max_dev;
details.f_final_dev = m.f_final_dev;
details.zeta = m.zeta;
details.T_settle = m.T_settle;
details.rocof = m.rocof_max;

details.H1 = m.I_peak_pu <= c.I_peak_pu_max;
details.H2 = m.f_max_dev <= c.f_dev_transient_Hz;
details.H3 = m.f_final_dev <= c.f_dev_final_Hz;
details.H4 = m.zeta >= c.zeta_min;
details.H5 = m.T_settle <= c.T_settle_max_s;
details.H6 = m.rocof_max <= c.rocof_max_Hzps;

is_stable = details.H1 && details.H2 && details.H3 && details.H4 && details.H5 && details.H6;
if ~is_stable
    score = 0; return;
end

w = config.weights;
S1 = max(0, min(1, 1 - (m.I_peak_pu-1)/(c.I_peak_pu_max-1+eps)));
S2 = max(0, min(1, 1 - m.f_max_dev/c.f_dev_transient_Hz));
S3 = max(0, min(1, 1 - m.f_final_dev/c.f_dev_final_Hz));
S4 = max(0, min(1, (m.zeta-c.zeta_min)/(0.20-c.zeta_min)));
S5 = max(0, min(1, 1 - m.T_settle/c.T_settle_max_s));
S6 = max(0, min(1, 1 - m.rocof_max/c.rocof_max_Hzps));
score = w.current*S1 + w.frequency*S2 + w.finalBias*S3 + w.damping*S4 + w.settling*S5 + w.rocof*S6;
end

function [is_stable, score, details] = fail_stub()
is_stable = false; score = 0;
details = struct('I_peak',NaN,'f_max_dev',NaN,'f_final_dev',NaN,'zeta',NaN,'T_settle',NaN,'rocof',NaN, ...
    'H1',false,'H2',false,'H3',false,'H4',false,'H5',false,'H6',false);
end

function summary = summarize_results(results, strategies)
for s = 1:numel(strategies)
    st = strategies{s};
    stable = [results.(['stable_' st])];
    sc = [results.(['score_' st])];
    summary.(st).n_stable = sum(stable);
    summary.(st).rate = 100*mean(stable);
    summary.(st).mean_score = mean(sc(stable)); if isnan(summary.(st).mean_score), summary.(st).mean_score = 0; end
end
end

function print_summary(summary, nCases)
fprintf('\n%-12s %10s %10s %12s\n','Strategy','Stable','Rate(%)','MeanScore');
fields = fieldnames(summary);
for i = 1:numel(fields)
    st = fields{i};
    fprintf('%-12s %5d/%-4d %9.1f %12.3f\n', st, summary.(st).n_stable, nCases, summary.(st).rate, summary.(st).mean_score);
end
end

function generate_figures_v6(results, summary, strategies, config, methods)
% 1 rate bar, 2 mean score, 3 heatmaps, 4 by SCR zone, 5 failures,
% 6 rocof boxplot, 7 zeta CDF, 8 taus map, 9 tspll map, 10 pareto scatter
nS = numel(strategies);
SCR = config.SCR_list; RX = config.RX_list; nCases = numel(results);

rates = zeros(1,nS); ms = zeros(1,nS);
for s = 1:nS
    rates(s)=summary.(strategies{s}).rate;
    ms(s)=summary.(strategies{s}).mean_score;
end

f=figure('Position',[50 50 840 420]); bar(rates); set(gca,'XTickLabel',strategies); ylabel('Stable rate (%)'); title('Figure1 Stability rate'); grid on;
saveas(f, fullfile(config.output_dir,'fig1_rate.png'));

f=figure('Position',[50 50 840 420]); bar(ms); set(gca,'XTickLabel',strategies); ylabel('Mean score'); title('Figure2 Mean score'); grid on;
saveas(f, fullfile(config.output_dir,'fig2_meanscore.png'));

f=figure('Position',[30 30 1600 300]);
for s=1:nS
    st=strategies{s}; M=zeros(numel(RX),numel(SCR));
    for k=1:nCases
        is = find(SCR==results(k).SCR,1); ir = find(abs(RX-results(k).RX)<1e-9,1);
        M(ir,is)=results(k).(['score_' st]);
    end
    subplot(1,nS,s); imagesc(SCR,RX,M); set(gca,'YDir','normal'); caxis([0 1]); colorbar; title(st); xlabel('SCR'); if s==1, ylabel('R/X'); end
end
sgtitle('Figure3 Score heatmaps'); saveas(f, fullfile(config.output_dir,'fig3_heatmaps.png'));

f=figure('Position',[50 50 900 420]);
weak=[results.SCR]<=1.5; med=[results.SCR]>1.5 & [results.SCR]<=4; strong=[results.SCR]>4;
B=zeros(3,nS);
for s=1:nS
    st=strategies{s}; sv=[results.(['stable_' st])];
    B(:,s)=[100*mean(sv(weak));100*mean(sv(med));100*mean(sv(strong))];
end
bar(B); legend(strategies,'Location','northwest'); set(gca,'XTickLabel',{'weak','med','strong'}); ylabel('Stable(%)'); title('Figure4 by grid strength'); grid on;
saveas(f, fullfile(config.output_dir,'fig4_gridstrength.png'));

f=figure('Position',[50 50 1000 420]); F=zeros(6,nS);
for s=1:nS
    st=strategies{s};
    for h=1:6
        F(h,s)=sum(~[results.(['H' num2str(h) '_' st])]);
    end
end
bar(F'); legend({'H1','H2','H3','H4','H5','H6'}); set(gca,'XTickLabel',strategies); title('Figure5 criteria failures'); grid on;
saveas(f, fullfile(config.output_dir,'fig5_failures.png'));

f=figure('Position',[50 50 900 420]); hold on;
for s=1:nS
    st=strategies{s}; r=[results.(['rocof_' st])]; boxchart(s*ones(size(r)), r);
end
set(gca,'XTick',1:nS,'XTickLabel',strategies); ylabel('RoCoF (Hz/s)'); title('Figure6 RoCoF distribution'); grid on;
saveas(f, fullfile(config.output_dir,'fig6_rocof_box.png'));

f=figure('Position',[50 50 900 420]); hold on;
for s=1:nS
    st=strategies{s}; z=[results.(['zeta_' st])]; z=z(isfinite(z));
    [f1,x1]=ecdf(z); plot(x1,f1,'LineWidth',1.5);
end
legend(strategies,'Location','southeast'); xlabel('zeta'); ylabel('CDF'); title('Figure7 Damping CDF'); grid on;
saveas(f, fullfile(config.output_dir,'fig7_zeta_cdf.png'));

% parameter surfaces (GPR as example)
f=figure('Position',[50 50 1000 400]);
[SS,RR]=meshgrid(SCR,RX); TA=zeros(size(SS)); TP=zeros(size(SS));
for i=1:size(SS,1)
    for j=1:size(SS,2)
        [TA(i,j),TP(i,j)] = get_method_params(methods,'GPR_AI',SS(i,j),RR(i,j));
    end
end
subplot(1,2,1); surf(SS,RR,TA*1e3); shading interp; xlabel('SCR'); ylabel('R/X'); zlabel('taus (ms)'); title('Figure8 GPR taus map');
subplot(1,2,2); surf(SS,RR,TP*1e3); shading interp; xlabel('SCR'); ylabel('R/X'); zlabel('tspll (ms)'); title('Figure9 GPR tspll map');
saveas(f, fullfile(config.output_dir,'fig8_9_param_surfaces.png'));

f=figure('Position',[50 50 900 420]); hold on;
for s=1:nS
    st=strategies{s};
    x=[results.(['f_max_dev_' st])]; y=[results.(['I_peak_' st])];
    scatter(x,y,22,'filled','DisplayName',st,'MarkerFaceAlpha',0.5);
end
xlabel('max |Δf| (Hz)'); ylabel('I_peak (p.u.)'); title('Figure10 Pareto: frequency deviation vs current peak'); grid on; legend('Location','best');
saveas(f, fullfile(config.output_dir,'fig10_pareto.png'));
end


function generate_small_signal_figures_v6(results, strategies, config)
% Proxy small-signal assessment from disturbance ring-down metrics
% NOTE: This is a data-driven proxy (zeta, settling, RoCoF, final bias) and
% not a full eigenvalue/impedance linearization workflow.

nS = numel(strategies);
SCR = config.SCR_list; RX = config.RX_list; nCases = numel(results);

% Figure SS1: zeta heatmaps for each strategy
f=figure('Position',[30 30 1600 300]);
for s=1:nS
    st=strategies{s}; Z=zeros(numel(RX),numel(SCR));
    for k=1:nCases
        is = find(SCR==results(k).SCR,1); ir = find(abs(RX-results(k).RX)<1e-9,1);
        Z(ir,is)=results(k).(['zeta_' st]);
    end
    subplot(1,nS,s); imagesc(SCR,RX,Z); set(gca,'YDir','normal'); colorbar;
    caxis([-0.05 0.35]); xlabel('SCR'); if s==1, ylabel('R/X'); end
    title([st ' zeta map']);
end
sgtitle('SS1: Damping-ratio maps (proxy small-signal)');
saveas(f, fullfile(config.output_dir,'ss1_zeta_maps.png'));

% Figure SS2: margin index boxplot
% margin > 0 indicates all hard criteria passed with average normalized headroom
f=figure('Position',[50 50 900 420]); hold on;
for s=1:nS
    st=strategies{s};
    M = compute_margin_vector(results, st, config);
    boxchart(s*ones(size(M)), M);
end
set(gca,'XTick',1:nS,'XTickLabel',strategies); ylabel('Margin index');
title('SS2: Composite stability-margin distribution'); grid on;
saveas(f, fullfile(config.output_dir,'ss2_margin_box.png'));

% Figure SS3: settling-vs-damping scatter
f=figure('Position',[50 50 900 420]); hold on;
for s=1:nS
    st=strategies{s};
    z=[results.(['zeta_' st])]; ts=[results.(['T_settle_' st])];
    scatter(z, ts, 18, 'filled', 'MarkerFaceAlpha',0.45, 'DisplayName', st);
end
xlabel('zeta'); ylabel('T_{settle} (s)'); title('SS3: Settling time vs damping');
legend('Location','best'); grid on;
saveas(f, fullfile(config.output_dir,'ss3_settling_vs_zeta.png'));

% Figure SS4: weak-grid sensitivity (SCR<=1.5)
weak = [results.SCR] <= 1.5;
f=figure('Position',[50 50 900 420]); vals=zeros(sum(weak),nS);
for s=1:nS
    st=strategies{s};
    vals(:,s) = [results(weak).(['rocof_' st])]';
end
bar(mean(vals,1,'omitnan'));
set(gca,'XTick',1:nS,'XTickLabel',strategies); ylabel('Mean RoCoF in weak grid (Hz/s)');
title('SS4: Weak-grid dynamic sensitivity'); grid on;
saveas(f, fullfile(config.output_dir,'ss4_weakgrid_rocof.png'));

fprintf('Generated 4 additional small-signal proxy figures (SS1-SS4)\n');
end

function M = compute_margin_vector(results, st, config)
c = config.criteria;
I = [results.(['I_peak_' st])];
Fmax = [results.(['f_max_dev_' st])];
Ffin = [results.(['f_final_dev_' st])];
Z = [results.(['zeta_' st])];
Ts = [results.(['T_settle_' st])];
R = [results.(['rocof_' st])];

m1 = 1 - (I./c.I_peak_pu_max);
m2 = 1 - (Fmax./c.f_dev_transient_Hz);
m3 = 1 - (Ffin./c.f_dev_final_Hz);
m4 = (Z - c.zeta_min)./max(c.zeta_min,eps);
m5 = 1 - (Ts./c.T_settle_max_s);
m6 = 1 - (R./c.rocof_max_Hzps);
M = mean([m1(:),m2(:),m3(:),m4(:),m5(:),m6(:)],2,'omitnan');
end
