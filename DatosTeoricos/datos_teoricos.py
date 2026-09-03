"""
Steady-state characteristic curves for the 57BLDC75-20730-08B21.

Casos incluidos:

1. Características del motor a 24 V frente al torque de carga.
2. Barrido de voltaje desde 6 V hasta 24 V.
3. Para el barrido 6-24 V:
       - Corriente máxima permitida = 1 A.
       - El voltaje aumenta a razón de 1 V cada 10 segundos.
       - 6 V  ->   0 s
       - 7 V  ->  10 s
       - 8 V  ->  20 s
       - ...
       - 24 V -> 180 s

Además se generan:

    tiempo_vs_rpm.csv
        t_s,Rpm

    tiempo_vs_voltaje.csv
        t_s,Voltaje
"""

from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


# ============================================================================
# MOTOR
# ============================================================================

@dataclass(frozen=True)
class Motor:

    model: str = "57BLDC75-20730-08B21"

    resistance: float = 0.48              # ohm
    torque_constant: float = 0.06         # N*m/A
    back_emf_v_per_krpm: float = 6.6      # V/kRPM

    no_load_current: float = 0.3          # A
    no_load_speed_rpm: float = 4000.0     # RPM

    nominal_voltage: float = 24.0         # V

    # Límite utilizado para las curvas de torque
    current_limit: float = 0.9            # A

    # Límite específico para el ensayo de 6 V a 24 V
    voltage_sweep_current_limit: float = 1.0  # A

    @property
    def back_emf_constant(self) -> float:
        """
        Back-EMF constant in V/(rad/s).
        """
        return self.back_emf_v_per_krpm / (
            1000.0 * 2.0 * np.pi / 60.0
        )

    @property
    def viscous_friction(self) -> float:
        """
        Viscous friction inferred from:

            Kt * i0 = B * omega0
        """

        omega0 = (
            self.no_load_speed_rpm
            * 2.0
            * np.pi
            / 60.0
        )

        return (
            self.torque_constant
            * self.no_load_current
            / omega0
        )


# ============================================================================
# TORQUE CHARACTERISTICS AT 24 V
# ============================================================================

def torque_characteristics(
    motor: Motor,
    torque_step: float = 1.0e-5,
) -> dict[str, np.ndarray]:

    r = motor.resistance
    kt = motor.torque_constant
    ke = motor.back_emf_constant
    b = motor.viscous_friction

    voltage_command = motor.nominal_voltage
    imax = motor.current_limit

    # ------------------------------------------------------------------------
    # CURRENT LIMIT TRANSITION
    # ------------------------------------------------------------------------

    omega_limit = (
        voltage_command
        - r * imax
    ) / ke

    omega_limit = max(
        omega_limit,
        0.0
    )

    current_limit_torque = (
        kt * imax
        - b * omega_limit
    )

    current_limit_torque = max(
        current_limit_torque,
        0.0
    )

    current_limit_rpm = (
        omega_limit
        * 60.0
        / (2.0 * np.pi)
    )

    # ------------------------------------------------------------------------
    # STALL
    # ------------------------------------------------------------------------

    stall_torque = (
        kt * imax
    )

    # ------------------------------------------------------------------------
    # TORQUE VECTOR
    # ------------------------------------------------------------------------

    load_torque = np.arange(
        0.0,
        stall_torque + torque_step,
        torque_step,
    )

    load_torque = load_torque[
        load_torque <= stall_torque + 1e-12
    ]

    if load_torque[-1] < stall_torque:
        load_torque = np.append(
            load_torque,
            stall_torque
        )

    n = len(load_torque)

    current = np.zeros(n)
    omega = np.zeros(n)
    effective_voltage = np.zeros(n)

    current_limited = np.zeros(
        n,
        dtype=bool
    )

    # ------------------------------------------------------------------------
    # OPERATING POINTS
    # ------------------------------------------------------------------------

    for k, torque in enumerate(load_torque):

        omega_unlimited = (
            voltage_command
            - (r / kt) * torque
        ) / (
            ke
            + r * b / kt
        )

        omega_unlimited = max(
            omega_unlimited,
            0.0
        )

        current_unlimited = (
            torque
            + b * omega_unlimited
        ) / kt

        # --------------------------------------------------------------------
        # NORMAL REGION
        # --------------------------------------------------------------------

        if current_unlimited <= imax:

            current[k] = current_unlimited
            omega[k] = omega_unlimited
            effective_voltage[k] = voltage_command

        # --------------------------------------------------------------------
        # CURRENT LIMITED REGION
        # --------------------------------------------------------------------

        else:

            current_limited[k] = True

            current[k] = imax

            omega[k] = (
                kt * imax
                - torque
            ) / b

            omega[k] = max(
                omega[k],
                0.0
            )

            effective_voltage[k] = (
                r * imax
                + ke * omega[k]
            )

            effective_voltage[k] = np.clip(
                effective_voltage[k],
                0.0,
                voltage_command,
            )

    # ------------------------------------------------------------------------
    # OUTPUT VARIABLES
    # ------------------------------------------------------------------------

    rpm = (
        omega
        * 60.0
        / (2.0 * np.pi)
    )

    mechanical_power = (
        load_torque
        * omega
    )

    electrical_power = (
        effective_voltage
        * current
    )

    efficiency = np.divide(
        100.0 * mechanical_power,
        electrical_power,
        out=np.full_like(
            mechanical_power,
            np.nan
        ),
        where=electrical_power > 1e-12,
    )

    return {

        "load_torque_Nm": load_torque,

        "current_A": current,

        "speed_RPM": rpm,

        "effective_voltage_V": effective_voltage,

        "mechanical_power_W": mechanical_power,

        "electrical_power_W": electrical_power,

        "efficiency_percent": efficiency,

        "current_limited": current_limited,

        "current_limit_torque_Nm":
            current_limit_torque,

        "current_limit_speed_RPM":
            current_limit_rpm,

        "stall_torque_Nm":
            stall_torque,

        "stall_current_A":
            imax,
    }


# ============================================================================
# VOLTAGE SWEEP
# 6 V -> 24 V
# 1 V every 10 seconds
# Maximum current = 1 A
# ============================================================================

def voltage_speed_characteristic(
    motor: Motor,
    minimum_voltage: float = 6.0,
    samples: int = 500,
    seconds_per_volt: float = 10.0,
) -> dict[str, np.ndarray]:

    maximum_voltage = motor.nominal_voltage

    # ------------------------------------------------------------------------
    # VOLTAGE VECTOR
    # ------------------------------------------------------------------------

    voltage = np.linspace(
        minimum_voltage,
        maximum_voltage,
        samples,
    )

    # ------------------------------------------------------------------------
    # TIME VECTOR
    #
    # 1 V = 10 seconds
    #
    # t = (V - 6) * 10
    #
    # 6 V  ->   0 s
    # 7 V  ->  10 s
    # ...
    # 24 V -> 180 s
    # ------------------------------------------------------------------------

    time_s = (
        voltage
        - minimum_voltage
    ) * seconds_per_volt

    # ------------------------------------------------------------------------
    # MOTOR EQUATIONS
    # ------------------------------------------------------------------------

    denominator = (
        motor.back_emf_constant
        + motor.resistance
        * motor.viscous_friction
        / motor.torque_constant
    )

    omega = (
        voltage
        / denominator
    )

    current = (
        motor.viscous_friction
        * omega
        / motor.torque_constant
    )

    # ------------------------------------------------------------------------
    # 1 A MAXIMUM CURRENT LIMIT
    # ------------------------------------------------------------------------

    current_limit = (
        motor.voltage_sweep_current_limit
    )

    current_limited = (
        current > current_limit
    )

    # If the theoretical point exceeds 1 A,
    # current is limited to exactly 1 A.

    for k in range(len(voltage)):

        if current[k] > current_limit:

            current[k] = current_limit

            # Mechanical equilibrium at zero external load:
            #
            # Kt*i = B*omega

            omega_from_current_limit = (
                motor.torque_constant
                * current_limit
                / motor.viscous_friction
            )

            # Electrical equilibrium:
            #
            # omega = (V - R*i)/Ke

            omega_from_voltage = (
                voltage[k]
                - motor.resistance
                * current_limit
            ) / motor.back_emf_constant

            omega[k] = max(
                0.0,
                min(
                    omega_from_current_limit,
                    omega_from_voltage
                )
            )

    # ------------------------------------------------------------------------
    # RPM
    # ------------------------------------------------------------------------

    speed_rpm = (
        omega
        * 60.0
        / (2.0 * np.pi)
    )

    return {

        "time_s": time_s,

        "voltage_V": voltage,

        "speed_RPM": speed_rpm,

        "current_A": current,

        "current_limited": current_limited,
    }


# ============================================================================
# GENERAL CSV
# ============================================================================

def save_csv(
    data: dict[str, np.ndarray],
    destination: Path,
) -> None:

    arrays = {

        name: value

        for name, value in data.items()

        if isinstance(
            value,
            np.ndarray
        )

        and value.ndim == 1
    }

    # Only arrays having the same length
    # as the first array are saved.

    if not arrays:
        return

    first_length = len(
        next(iter(arrays.values()))
    )

    arrays = {

        name: value

        for name, value in arrays.items()

        if len(value) == first_length
    }

    headers = list(arrays)

    matrix = np.column_stack(
        [
            arrays[name]
            for name in headers
        ]
    )

    np.savetxt(
        destination,
        matrix,
        delimiter=",",
        header=",".join(headers),
        comments="",
        fmt="%.10g",
    )


# ============================================================================
# SAVE TIME VS RPM
# ============================================================================

def save_time_vs_rpm_csv(
    voltage_data: dict[str, np.ndarray],
    destination: Path,
) -> None:

    matrix = np.column_stack(
        (
            voltage_data["time_s"],
            voltage_data["speed_RPM"],
        )
    )

    np.savetxt(
        destination,
        matrix,
        delimiter=",",
        header="t_s,Rpm",
        comments="",
        fmt="%.10g",
    )


# ============================================================================
# SAVE TIME VS VOLTAGE
# ============================================================================

def save_time_vs_voltage_csv(
    voltage_data: dict[str, np.ndarray],
    destination: Path,
) -> None:

    matrix = np.column_stack(
        (
            voltage_data["time_s"],
            voltage_data["voltage_V"],
        )
    )

    np.savetxt(
        destination,
        matrix,
        delimiter=",",
        header="t_s,Voltaje",
        comments="",
        fmt="%.10g",
    )


# ============================================================================
# PLOTS
# ============================================================================

def plot_curves(
    motor: Motor,
    torque_data: dict[str, np.ndarray],
    voltage_data: dict[str, np.ndarray],
    torque_destination: Path,
    voltage_destination: Path,
) -> None:

    torque = torque_data["load_torque_Nm"]

    limited = torque_data["current_limited"]

    normal = ~limited

    limit_torque = (
        torque_data[
            "current_limit_torque_Nm"
        ]
    )

    # ------------------------------------------------------------------------
    # STYLE
    # ------------------------------------------------------------------------

    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.size": 10,
            "axes.titlesize": 12,
            "axes.labelsize": 11,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "axes.linewidth": 0.8,
            "savefig.dpi": 300,
        }
    )

    speed_color = "#1f77b4"
    current_color = "#d62728"
    power_color = "#2ca02c"
    efficiency_color = "#9467bd"
    voltage_color = "#ff7f0e"

    # ========================================================================
    # FIGURE 1
    # ========================================================================

    fig1, axes = plt.subplots(
        2,
        2,
        figsize=(12.5, 8.5),
        constrained_layout=True,
    )

    def draw_curve(
        ax,
        y,
        color,
        ylabel,
        title,
    ):

        ax.plot(
            torque[normal],
            y[normal],
            color=color,
            linestyle="-",
            linewidth=2.3,
        )

        if np.any(limited):

            first_limited = int(
                np.argmax(limited)
            )

            start_idx = max(
                first_limited - 1,
                0
            )

            ax.plot(
                torque[start_idx:],
                y[start_idx:],
                color=color,
                linestyle="--",
                linewidth=2.3,
            )

        ax.axvline(
            limit_torque,
            color="0.30",
            linestyle=":",
            linewidth=1.2,
        )

        ax.set_xlabel(
            "Load torque, $T_L$ [N·m]"
        )

        ax.set_ylabel(
            ylabel
        )

        ax.set_title(
            title,
            fontweight="semibold"
        )

        ax.grid(
            True,
            which="major",
            linestyle="--",
            linewidth=0.6,
            alpha=0.30,
        )

        ax.minorticks_on()

        ax.grid(
            True,
            which="minor",
            linestyle=":",
            linewidth=0.4,
            alpha=0.12,
        )

        ax.tick_params(
            direction="in",
            top=True,
            right=True,
        )

    # ------------------------------------------------------------------------
    # SPEED
    # ------------------------------------------------------------------------

    draw_curve(
        axes[0, 0],
        torque_data["speed_RPM"],
        speed_color,
        "Speed [RPM]",
        "Rotor Speed vs Load Torque",
    )

    # ------------------------------------------------------------------------
    # CURRENT
    # ------------------------------------------------------------------------

    draw_curve(
        axes[0, 1],
        torque_data["current_A"],
        current_color,
        "Current [A]",
        "Motor Current vs Load Torque",
    )

    axes[0, 1].axhline(
        motor.current_limit,
        color=current_color,
        linestyle=":",
        linewidth=1.2,
    )

    axes[0, 1].set_ylim(
        0.0,
        1.08 * motor.current_limit,
    )

    axes[0, 1].annotate(
        f"$I_{{max}}={motor.current_limit:.1f}$ A",
        xy=(
            torque[-1],
            motor.current_limit,
        ),
        xytext=(-8, -10),
        textcoords="offset points",
        ha="right",
        va="top",
        fontsize=9,
        color=current_color,
    )

    # ------------------------------------------------------------------------
    # POWER
    # ------------------------------------------------------------------------

    draw_curve(
        axes[1, 0],
        torque_data["mechanical_power_W"],
        power_color,
        "Mechanical Power [W]",
        "Mechanical Power vs Load Torque",
    )

    # ------------------------------------------------------------------------
    # EFFICIENCY
    # ------------------------------------------------------------------------

    draw_curve(
        axes[1, 1],
        torque_data["efficiency_percent"],
        efficiency_color,
        "Efficiency [%]",
        "Efficiency vs Load Torque",
    )

    from matplotlib.lines import Line2D

    legend_elements = [

        Line2D(
            [0],
            [0],
            color="black",
            linestyle="-",
            linewidth=2.2,
            label="Voltage-controlled region",
        ),

        Line2D(
            [0],
            [0],
            color="black",
            linestyle="--",
            linewidth=2.2,
            label=(
                f"Current-limited region "
                f"($I={motor.current_limit:.1f}$ A)"
            ),
        ),

        Line2D(
            [0],
            [0],
            color="0.30",
            linestyle=":",
            linewidth=1.2,
            label="Current-limit transition",
        ),
    ]

    fig1.legend(
        handles=legend_elements,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.08),
        ncol=3,
        frameon=False,
    )

    fig1.savefig(
        torque_destination,
        bbox_inches="tight",
        facecolor="white",
    )

    plt.close(fig1)

    # ========================================================================
    # FIGURE 2
    # SPEED VS VOLTAGE
    # ========================================================================

    fig2, ax = plt.subplots(
        figsize=(9.5, 6.5),
        constrained_layout=True,
    )

    voltage = (
        voltage_data["voltage_V"]
    )

    speed_rpm = (
        voltage_data["speed_RPM"]
    )

    ax.plot(
        voltage,
        speed_rpm,
        color=voltage_color,
        linewidth=3.2,
    )

    min_voltage = float(
        voltage[0]
    )

    min_speed = float(
        speed_rpm[0]
    )

    max_voltage = float(
        voltage[-1]
    )

    max_speed = float(
        speed_rpm[-1]
    )

    ax.scatter(
        [
            min_voltage,
            max_voltage
        ],
        [
            min_speed,
            max_speed
        ],
        s=80,
        color=voltage_color,
        edgecolor="black",
        linewidth=0.7,
        zorder=5,
    )

    ax.annotate(
        f"{min_speed:.0f} RPM",
        xy=(
            min_voltage,
            min_speed
        ),
        xytext=(23, 3),
        textcoords="offset points",
        ha="left",
        va="top",
        fontsize=18,
    )

    ax.annotate(
        f"{max_speed:.0f} RPM",
        xy=(
            max_voltage,
            max_speed
        ),
        xytext=(-15, 15),
        textcoords="offset points",
        ha="right",
        va="top",
        fontsize=18,
    )

    ax.set_xlim(
        6.0,
        24.0
    )

    ax.set_xticks(
        np.arange(
            6.0,
            24.1,
            2.0
        )
    )

    y_padding = (
        0.08
        * (
            max_speed
            - min_speed
        )
    )

    ax.set_ylim(
        max(
            0.0,
            min_speed - y_padding
        ),
        max_speed
        + 1.5 * y_padding,
    )

    ax.set_xlabel(
        "Motor Voltage [V]",
        fontsize=23,
        labelpad=10,
    )

    ax.set_ylabel(
        "Speed [RPM]",
        fontsize=23,
        labelpad=10,
    )

    ax.set_title(
        "No-Load Speed vs Motor Voltage",
        fontsize=26,
        fontweight="bold",
        pad=16,
    )

    ax.grid(
        True,
        which="major",
        linestyle="--",
        linewidth=0.6,
        alpha=0.30,
    )

    ax.minorticks_on()

    ax.grid(
        True,
        which="minor",
        linestyle=":",
        linewidth=0.4,
        alpha=0.12,
    )

    ax.tick_params(
        direction="in",
        top=True,
        right=True,
        labelsize=20,
        length=6,
        width=1.0,
    )

    fig2.savefig(
        voltage_destination,
        bbox_inches="tight",
        facecolor="white",
    )

    plt.close(fig2)


# ============================================================================
# MAIN
# ============================================================================

def main() -> None:

    motor = Motor()

    # ------------------------------------------------------------------------
    # OUTPUT DIRECTORY
    # ------------------------------------------------------------------------

    output_directory = (
        Path(__file__).resolve().parent
        / "steady_state_results"
    )

    output_directory.mkdir(
        exist_ok=True
    )

    # ------------------------------------------------------------------------
    # CALCULATE TORQUE CURVES
    # ------------------------------------------------------------------------

    torque_data = torque_characteristics(
        motor,
        torque_step=1.0e-5,
    )

    # ------------------------------------------------------------------------
    # CALCULATE 6 V -> 24 V SWEEP
    #
    # 1 V every 10 seconds
    #
    # Total time:
    #
    # (24 - 6) * 10 = 180 seconds
    # ------------------------------------------------------------------------

    voltage_data = voltage_speed_characteristic(
        motor,
        minimum_voltage=6.0,
        samples=500,
        seconds_per_volt=10.0,
    )

    # ------------------------------------------------------------------------
    # SAVE ORIGINAL DATA
    # ------------------------------------------------------------------------

    save_csv(
        torque_data,
        output_directory
        / "torque_characteristics.csv",
    )

    save_csv(
        voltage_data,
        output_directory
        / "voltage_speed_characteristic.csv",
    )

    # ------------------------------------------------------------------------
    # SAVE REQUIRED CSV FILES
    # ------------------------------------------------------------------------

    save_time_vs_rpm_csv(
        voltage_data,
        output_directory
        / "tiempo_vs_rpm.csv",
    )

    save_time_vs_voltage_csv(
        voltage_data,
        output_directory
        / "tiempo_vs_voltaje.csv",
    )

    # ------------------------------------------------------------------------
    # PLOTS
    # ------------------------------------------------------------------------

    plot_curves(
        motor,
        torque_data,
        voltage_data,
        output_directory
        / "motor_characteristics_vs_torque.png",
        output_directory
        / "speed_vs_voltage.png",
    )

    # ------------------------------------------------------------------------
    # PRINT RESULTS
    # ------------------------------------------------------------------------

    print(
        "============================================================"
    )

    print(
        f"Motor: {motor.model}"
    )

    print(
        "============================================================"
    )

    print(
        f"Ke = "
        f"{motor.back_emf_constant:.8f} V/(rad/s)"
    )

    print(
        f"B  = "
        f"{motor.viscous_friction:.8e} N*m*s/rad"
    )

    print()

    print(
        "DRIVER LIMIT - TORQUE TEST"
    )

    print(
        "------------------------------------------------------------"
    )

    print(
        f"Current limit = "
        f"{motor.current_limit:.3f} A"
    )

    print(
        f"Load torque at current limit = "
        f"{torque_data['current_limit_torque_Nm']:.6f} N*m"
    )

    print(
        f"Speed at current limit = "
        f"{torque_data['current_limit_speed_RPM']:.2f} RPM"
    )

    print()

    print(
        "VOLTAGE SWEEP"
    )

    print(
        "------------------------------------------------------------"
    )

    print(
        "Voltage range = 6 V -> 24 V"
    )

    print(
        "Voltage rate = 1 V every 10 s"
    )

    print(
        f"Total sweep time = "
        f"{voltage_data['time_s'][-1]:.1f} s"
    )

    print(
        f"Maximum current = "
        f"{motor.voltage_sweep_current_limit:.1f} A"
    )

    print()

    print(
        "CSV FILES"
    )

    print(
        "------------------------------------------------------------"
    )

    print(
        "tiempo_vs_rpm.csv"
    )

    print(
        "Header: t_s,Rpm"
    )

    print()

    print(
        "tiempo_vs_voltaje.csv"
    )

    print(
        "Header: t_s,Voltaje"
    )

    print()

    print(
        f"Results saved in: "
        f"{output_directory}"
    )


# ============================================================================
# RUN
# ============================================================================

if __name__ == "__main__":
    main()