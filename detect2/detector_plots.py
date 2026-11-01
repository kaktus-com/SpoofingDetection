"""Scientific figures for oracle detector fits and held-out label evaluation."""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def setup_style():
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9,
                         "axes.titlesize": 10, "axes.labelsize": 9,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "axes.grid": True, "grid.alpha": .2,
                         "savefig.dpi": 150, "legend.fontsize": 8})


def truth_for(evaluation, run, index):
    for row in evaluation.get("windows", []):
        if row["scenario"] == run["scenario"] and row["case"] == run["case"] and row["window_index"] == index:
            return row["truth"]
    return "unavailable"


def decisions_plot(name, runs, evaluation):
    fig, axes = plt.subplots(3, len(runs), figsize=(6*len(runs), 9), squeeze=False,
                              sharex="col", layout="constrained")
    states = {"benign": 1, "inconclusive": 0, "spoofed": -1}
    for col, run in enumerate(runs):
        records = run["records"]
        t = np.array([.5*(record.start_s+record.end_s) for record in records])
        for hypothesis, color in [("benign", "#2563a6"), ("spoofed", "#d56824")]:
            costs = [getattr(record.result, hypothesis).reduced_cost if getattr(record.result, hypothesis) else np.nan for record in records]
            axes[0, col].plot(t, costs, "o-", color=color, label=hypothesis)
        axes[0, col].set(title=f"{run['case']}: informative QR costs", ylabel="Reduced negative log-likelihood")
        axes[0, col].legend()
        gamma = [record.result.gamma if record.result.gamma is not None else np.nan for record in records]
        axes[1, col].plot(t, gamma, "o-", color="#8b3bb0")
        axes[1, col].axhline(0, color="#475569", linewidth=.8)
        axes[1, col].set(ylabel="Gamma = J_sp − J_benign", title="Positive favors benign; negative favors spoofing")
        decisions = [states[record.result.decision] for record in records]
        axes[2, col].step(t, decisions, where="mid", label="Detector", color="#8b3bb0")
        axes[2, col].scatter(t, decisions, color="#8b3bb0", s=28)
        actual = [states.get(truth_for(evaluation, run, record.index), np.nan) for record in records]
        axes[2, col].plot(t, actual, "--", color="#475569", label="Held-out label")
        axes[2, col].set(xlabel="Window midpoint (s)", yticks=[-1, 0, 1],
                        yticklabels=["Spoofed", "Inconclusive", "Benign"], ylim=(-1.3, 1.3))
        axes[2, col].legend()
        reasons = sorted({record.result.reason for record in records if record.result.decision == 'inconclusive'})
        if all(record.result.gamma is None for record in records):
            for row in [0, 1]:
                axes[row, col].text(.5, .5, "No reliable 3D fit\n"+"\n".join(reasons), ha="center", va="center", transform=axes[row, col].transAxes)
    mode = runs[0].get("mode", "oracle")
    fig.suptitle(f"{name}: {mode}-displacement detector\nInconclusive results are retained; labels were read after prediction")
    return fig


def first_fitted(run):
    return next(record for record in run["records"] if record.result.benign is not None and record.result.spoofed is not None)


def fits_plot(run):
    record = first_fitted(run)
    window, result = record.data, record.result
    active = result.signal_indices
    fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True, sharey="row", layout="constrained")
    for col, name in enumerate(["benign", "spoofed"]):
        fit = getattr(result, name)
        for k, j in enumerate(active):
            color = plt.get_cmap("tab10")(k)
            raw_relative = window.phase_cycles[:, j]-window.phase_cycles[0, j]
            axes[0, col].plot(window.time_s, raw_relative, color=color, alpha=.4)
            axes[0, col].plot(window.time_s, fit.fitted_phase_cycles[:, k]-window.phase_cycles[0, j], "--", color=color)
            axes[1, col].plot(window.time_s, window.phase_cycles[:, j]-fit.trend_cycles[:, k], color=color, alpha=.4)
            axes[1, col].plot(window.time_s, fit.geometric_phase_cycles[:, k], "--", color=color, label=window.signal_ids[j])
            axes[2, col].plot(window.time_s, fit.residual_cycles[:, k], color=color)
        axes[0, col].set(title=f"{name} hypothesis: observation (solid), fit (dashed)", ylabel="Phase relative to first sample (cycles)")
        axes[1, col].set(title="Observation − ESTIMATED trend; fitted motion dashed", ylabel="Motion component (cycles)")
        axes[2, col].set(title="Observed − fitted phase", ylabel="Residual (cycles)", xlabel="Measurement time (s)")
    axes[1, 0].legend(ncol=4)
    fig.suptitle(f"{run['scenario']}/{run['case']}: window [{record.start_s:g}, {record.end_s:g}) s\n"
                 "Both hypotheses fit their own nuisance trends from raw phase; no generated components are used")
    return fig


def residuals_plot(run):
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), layout="constrained")
    records = [record for record in run["records"] if record.result.benign is not None and record.result.spoofed is not None]
    for col, name in enumerate(["benign", "spoofed"]):
        all_normalized = []
        for record in records:
            fit = getattr(record.result, name)
            all_normalized.extend(fit.normalized_residuals.ravel())
            for k in range(fit.residual_cycles.shape[1]):
                axes[0, col].plot(record.data.time_s, fit.residual_cycles[:, k], color=plt.get_cmap("tab10")(k), alpha=.6, linewidth=1)
        axes[0, col].set(title=f"{name} model: residuals over fitted windows", ylabel="Residual (cycles)", xlabel="Measurement time (s)")
        axes[1, col].hist(all_normalized, bins=35, density=True, alpha=.65, color="#8b3bb0")
        if not any(record.result.diagnostics.get("sigma_floor_applied") for record in records):
            x = np.linspace(-4, 4, 200)
            axes[1, col].plot(x, np.exp(-.5*x*x)/np.sqrt(2*np.pi), color="#475569", label="Unit-normal reference")
            axes[1, col].legend()
        axes[1, col].set(title="Whitened fitted residual distribution", xlabel="Whitened residual", ylabel="Density")
    fig.suptitle(f"{run['scenario']}/{run['case']}: per-window residual diagnostics\n"
                 "Fitting reduces residual degrees of freedom; deterministic sigma floors are not calibrated noise")
    return fig
