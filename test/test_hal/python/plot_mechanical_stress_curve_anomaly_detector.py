import numpy as np
import matplotlib.pyplot as plt



def plot_mechanical_stress_curve(a=5.0, b=0.04, threshold=40.0,
                                max_torque=200.0):
    """Plot allowed exposure time while outside joint position limits."""
    if a <= 0 or b < 0 or threshold < 0 or max_torque <= threshold:
        raise ValueError("Require a > 0, b >= 0, and max_torque > threshold >= 0.")

    torque = np.linspace(threshold, max_torque, 501)[1:]
    allowed_time = a * np.exp(-b * (torque - threshold))

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.axvspan(0, threshold, color="green", alpha=0.12,
               label="No mechanical-stress timing")
    ax.axvline(threshold, color="gray", linestyle="--")
    ax.plot(torque, allowed_time,
            label=rf"$t={a}\,e^{{-{b}(|\tau|-{threshold})}}$")

    ax.set(
        xlabel="Torque magnitude |τ| [Nm]",
        ylabel="Allowed continuous exposure [s]",
        title="Mechanical stress — position outside limits",
        xlim=(0, max_torque),
        ylim=(0, a * 1.1),
    )
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    plt.show()
    return ax


plot_mechanical_stress_curve()