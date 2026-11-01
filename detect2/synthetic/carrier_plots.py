"""Truth-aware diagnostics for synthetic observations, never detector decisions."""

import numpy as np

from .plots import COLORS, components, plt, setup_style
from .carrier import CASES


def traces(ax, pair, values, reference=False):
    for i, name in enumerate(pair.signal_ids):
        if reference and i == 0:
            continue
        ax.plot(pair.time_s, values[:, i], color=plt.get_cmap("tab10")(i), label=name)
    ax.set_xlabel("Measurement time (s)")


def observations_plot(pair):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True, sharey="row", layout="constrained")
    for col, name in enumerate(CASES):
        case = pair.cases[name]
        traces(axes[0, col], pair, case.phase_cycles-case.phase_cycles[0])
        axes[0, col].set(title=f"{name}: observed phase relative to its first sample", ylabel="Observed Δphase (cycles)")
        traces(axes[1, col], pair, case.phase_cycles-case.slow_phase_cycles)
        axes[1, col].set(title=f"{name}: observed phase − known generated trend", ylabel="Motion + noise (cycles)")
    axes[0, 0].legend(ncol=4)
    axes[1, 0].legend(ncol=4)
    fig.suptitle(f"{pair.imu_motion.config.scenario}: paired observations at {pair.config.gnss_rate_hz:g} Hz\n"
                 "Bottom row uses simulation truth for visualization; no trend estimator or detector is running")
    return fig


def components_plot(pair):
    fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True, sharey="row", layout="constrained")
    for col, name in enumerate(CASES):
        case = pair.cases[name]
        for row, (values, title) in enumerate([
                (case.geometric_phase_cycles, "Motion-induced phase"),
                (case.slow_phase_cycles-case.slow_phase_cycles[0], "Quadratic nuisance trend, constant removed"),
                (case.noise_cycles, "Independent noise realization")]):
            traces(axes[row, col], pair, values)
            axes[row, col].set(title=f"{name}: {title}", ylabel="Phase component (cycles)")
    axes[0, 0].legend(ncol=4)
    fig.suptitle(f"{pair.imu_motion.config.scenario}: truth decomposition\n"
                 "Observed phase = constant + slow change + geometric contribution + noise")
    return fig


def between_signals_plot(pair):
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True, sharey="row", layout="constrained")
    for col, name in enumerate(CASES):
        case = pair.cases[name]
        pure = case.geometric_phase_cycles
        observed = case.phase_cycles-case.slow_phase_cycles
        traces(axes[0, col], pair, pure-pure[:, :1], reference=True)
        traces(axes[1, col], pair, observed-observed[:, :1], reference=True)
        axes[0, col].set(title=f"{name}: geometric differences relative to S01", ylabel="Phase difference (cycles)")
        axes[1, col].set(title=f"{name}: noisy differences after truth-trend removal", ylabel="Phase difference (cycles)")
    axes[0, 0].legend(ncol=4)
    fig.suptitle(f"{pair.imu_motion.config.scenario}: cross-signal comparison\n"
                 "Shared-direction geometric contributions cancel; independent receiver noise does not")
    return fig


def motion_plot(pair):
    imu, gnss = pair.imu_motion, pair.gnss_motion
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), layout="constrained")
    stop = min(.5, float(imu.time_s[-1]))
    for t, row, name, color in [(imu.time_s, 0, "Ideal IMU", "#475569"),
                                 (gnss.time_s, 1, "Carrier epochs", "#8b3bb0")]:
        selected = t[t <= stop+1e-12]
        axes[0, 0].scatter(selected, np.full(len(selected), row), label=name, color=color, marker="|", s=90)
    axes[0, 0].set(yticks=[0, 1], yticklabels=["IMU", "GNSS"], ylim=(-.5, 1.5),
                   xlabel="Common measurement time (s)", title="Two sample grids; no interpolation or clock offset")
    displacement = imu.antenna_position_m-imu.antenna_position_m[0]
    at_gnss = gnss.antenna_position_m-gnss.antenna_position_m[0]
    components(axes[0, 1], imu.time_s, displacement)
    for i in range(3):
        axes[0, 1].scatter(gnss.time_s, at_gnss[:, i], color=COLORS[i], s=5)
    axes[0, 1].set(title="True antenna displacement; dots at GNSS epochs", ylabel="Displacement ENU (m)")
    components(axes[1, 0], imu.time_s, imu.specific_force_body_m_s2, ["Forward", "Right", "Down"])
    axes[1, 0].set(title="Shared ideal accelerometer input (specific force)", ylabel="Specific force (m/s²)")
    components(axes[1, 1], imu.time_s, imu.angular_rate_body_rad_s, ["Forward", "Right", "Down"])
    axes[1, 1].set(title="Shared ideal gyro input", ylabel="Angular rate (rad/s)")
    fig.suptitle(f"{imu.config.scenario}: {pair.config.gnss_rate_hz:g} Hz GNSS / {imu.config.rate_hz:g} Hz ideal IMU")
    return fig


def noise_plot(pair):
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), layout="constrained")
    sigma = pair.config.noise_sigma_cycles
    for col, name in enumerate(CASES):
        noise = pair.cases[name].noise_cycles
        if sigma:
            axes[0, col].hist(noise.ravel()/sigma, bins=30, density=True, color="#8b3bb0", alpha=.6)
            x = np.linspace(-4., 4., 200)
            axes[0, col].plot(x, np.exp(-.5*x*x)/np.sqrt(2*np.pi), color="#475569", label="N(0,1) marginal")
            axes[0, col].legend()
        else:
            axes[0, col].axvline(0., color="#8b3bb0", label="Noise disabled")
            axes[0, col].legend()
        axes[0, col].set(title=f"{name}: generated noise only (truth)",
                         xlabel="Noise / configured sigma" if sigma else "Noise (cycles)", ylabel="Density")
        lags = np.arange(min(11, len(noise)))
        centered = noise-noise.mean(axis=0)
        correlation = []
        for lag in lags:
            a, b = centered[lag:], centered[:len(noise)-lag]
            denominator = np.sqrt(np.sum(a*a)*np.sum(b*b))
            correlation.append(float(np.sum(a*b)/denominator) if denominator else 0.)
        axes[1, col].plot(lags, correlation, "o-", color="#8b3bb0", label="Sample estimate")
        if sigma:
            axes[1, col].plot(lags, pair.config.noise_rho**lags, "--", color="#475569", label="Configured AR(1)")
        axes[1, col].set(title=f"{name}: finite-record noise autocorrelation", xlabel="Lag (GNSS samples)", ylabel="Correlation", ylim=(-.3, 1.1))
        axes[1, col].legend()
    fig.suptitle(f"{pair.imu_motion.config.scenario}: sigma={sigma:g} cycles, rho={pair.config.noise_rho:g}\n"
                 "Diagnostics of known generated noise, not receiver quality estimates")
    return fig
