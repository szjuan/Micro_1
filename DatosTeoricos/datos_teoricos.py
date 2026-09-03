"""
Steady-state characteristic curves for the 57BLDC75-20730-08B21.

CASO 1:
    Características del motor a 24 V frente al torque de carga.

CASO 2:
    Barrido escalonado de voltaje:

        0 - 10 s      -> 6 V
        10 - 20 s     -> 7 V
        20 - 30 s     -> 8 V
        30 - 40 s     -> 9 V
        ...
        170 - 180 s   -> 23 V
        180 s         -> 24 V

    El ensayo EMPIEZA directamente en 6 V.
    NO existe un escalón inicial en 0 V.

    El voltaje aumenta 1 V cada 10 segundos.

    Corriente máxima para este ensayo = 1 A.

CSV generados:

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

    # Límite utilizado para las curvas de torque.
    current_limit: float = 0.9             # A

    # Límite específico del ensayo escalonado 6 -> 24 V.
    voltage_sweep_current_limit: float = 1.0   # A

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
# CASO 1
# CARACTERÍSTICAS DEL MOTOR VS TORQUE A 24 V
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
    # PUNTO DONDE COMIENZA EL LÍMITE DE CORRIENTE
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
    # TORQUE DE BLOQUEO
    # ------------------------------------------------------------------------

    stall_torque = (
        kt * imax
    )

    # ------------------------------------------------------------------------
    # VECTOR DE TORQUE
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
    # PUNTOS DE OPERACIÓN
    # ------------------------------------------------------------------------

    for k, torque in enumerate(load_torque):

        # Punto sin límite de corriente.

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
        # REGIÓN NORMAL
        # --------------------------------------------------------------------

        if current_unlimited <= imax:

            current[k] = current_unlimited

            omega[k] = omega_unlimited

            effective_voltage[k] = voltage_command

        # --------------------------------------------------------------------
        # REGIÓN LIMITADA POR CORRIENTE
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
    # RPM
    # ------------------------------------------------------------------------

    rpm = (
        omega
        * 60.0
        / (2.0 * np.pi)
    )

    # ------------------------------------------------------------------------
    # POTENCIA
    # ------------------------------------------------------------------------

    mechanical_power = (
        load_torque
        * omega
    )

    electrical_power = (
        effective_voltage
        * current
    )

    # ------------------------------------------------------------------------
    # EFICIENCIA
    # ------------------------------------------------------------------------

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

        "load_torque_Nm":
            load_torque,

        "current_A":
            current,

        "speed_RPM":
            rpm,

        "effective_voltage_V":
            effective_voltage,

        "mechanical_power_W":
            mechanical_power,

        "electrical_power_W":
            electrical_power,

        "efficiency_percent":
            efficiency,

        "current_limited":
            current_limited,

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
# CASO 2
#
# BARRIDO ESCALONADO:
#
# 0 - 10 s      -> 6 V
# 10 - 20 s     -> 7 V
# 20 - 30 s     -> 8 V
# 30 - 40 s     -> 9 V
# ...
# 170 - 180 s   -> 23 V
# 180 s         -> 24 V
#
# Imax = 1 A
#
# ============================================================================

def voltage_speed_characteristic(
    motor: Motor,
    minimum_voltage: float = 6.0,
    maximum_voltage: float = 24.0,
    voltage_step: float = 1.0,
    seconds_per_step: float = 10.0,
) -> dict[str, np.ndarray]:

    # ------------------------------------------------------------------------
    # NIVELES DE VOLTAJE
    #
    # 6, 7, 8, 9, ..., 24 V
    #
    # IMPORTANTE:
    # NO se agrega 0 V al inicio.
    # ------------------------------------------------------------------------

    voltage_levels = np.arange(
        minimum_voltage,
        maximum_voltage + voltage_step,
        voltage_step,
    )

    # ------------------------------------------------------------------------
    # CONSTRUCCIÓN DE ESCALONES
    #
    # El vector queda de esta forma:
    #
    # t_s    Voltaje
    #
    # 0      6
    # 10     6
    # 10     7
    # 20     7
    # 20     8
    # 30     8
    # 30     9
    # ...
    # 170    23
    # 180    23
    # 180    24
    #
    # Repetir el mismo tiempo en la transición genera
    # una línea vertical cuando se grafica.
    # ------------------------------------------------------------------------

    time_points = []
    voltage_points = []

    for k in range(len(voltage_levels) - 1):

        voltage_actual = voltage_levels[k]

        tiempo_inicio = (
            k * seconds_per_step
        )

        tiempo_final = (
            (k + 1)
            * seconds_per_step
        )

        # Inicio del escalón.
        time_points.append(
            tiempo_inicio
        )

        voltage_points.append(
            voltage_actual
        )

        # Fin del escalón.
        time_points.append(
            tiempo_final
        )

        voltage_points.append(
            voltage_actual
        )

    # ------------------------------------------------------------------------
    # ÚLTIMO PUNTO
    #
    # A los 180 segundos:
    #
    # 23 V -> 24 V
    # ------------------------------------------------------------------------

    final_time = (
        (len(voltage_levels) - 1)
        * seconds_per_step
    )

    time_points.append(
        final_time
    )

    voltage_points.append(
        voltage_levels[-1]
    )

    # ------------------------------------------------------------------------
    # CONVERTIR A NUMPY
    # ------------------------------------------------------------------------

    time_s = np.array(
        time_points,
        dtype=float
    )

    voltage = np.array(
        voltage_points,
        dtype=float
    )

    # ------------------------------------------------------------------------
    # PARÁMETROS DEL MOTOR
    # ------------------------------------------------------------------------

    r = motor.resistance
    kt = motor.torque_constant
    ke = motor.back_emf_constant
    b = motor.viscous_friction

    # Corriente máxima de este ensayo.
    imax = (
        motor.voltage_sweep_current_limit
    )

    # ------------------------------------------------------------------------
    # ARRAYS DE RESULTADOS
    # ------------------------------------------------------------------------

    omega = np.zeros_like(
        voltage
    )

    current = np.zeros_like(
        voltage
    )

    current_limited = np.zeros(
        len(voltage),
        dtype=bool
    )

    # ------------------------------------------------------------------------
    # MODELO EN ESTADO ESTACIONARIO
    # ------------------------------------------------------------------------

    denominator = (
        ke
        + r * b / kt
    )

    # ------------------------------------------------------------------------
    # CALCULAR RPM PARA CADA ESCALÓN
    # ------------------------------------------------------------------------

    for k, v in enumerate(voltage):

        # --------------------------------------------------------------------
        # PUNTO SIN LÍMITE DE CORRIENTE
        # --------------------------------------------------------------------

        omega_unlimited = (
            v / denominator
        )

        current_unlimited = (
            b
            * omega_unlimited
            / kt
        )

        # --------------------------------------------------------------------
        # REGIÓN NORMAL
        # --------------------------------------------------------------------

        if current_unlimited <= imax:

            omega[k] = omega_unlimited

            current[k] = current_unlimited

        # --------------------------------------------------------------------
        # REGIÓN LIMITADA A 1 A
        # --------------------------------------------------------------------

        else:

            current_limited[k] = True

            current[k] = imax

            # Límite mecánico debido a Imax.
            omega_mechanical_limit = (
                kt
                * imax
                / b
            )

            # Límite eléctrico debido al voltaje disponible.
            omega_electrical_limit = (
                v
                - r * imax
            ) / ke

            omega_electrical_limit = max(
                omega_electrical_limit,
                0.0
            )

            omega[k] = min(
                omega_mechanical_limit,
                omega_electrical_limit
            )

    # ------------------------------------------------------------------------
    # CONVERTIR RAD/S -> RPM
    # ------------------------------------------------------------------------

    speed_rpm = (
        omega
        * 60.0
        / (2.0 * np.pi)
    )

    return {

        "time_s":
            time_s,

        "voltage_V":
            voltage,

        "speed_RPM":
            speed_rpm,

        "current_A":
            current,

        "current_limited":
            current_limited,
    }


# ============================================================================
# GUARDAR CSV GENERAL
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
# CSV SOLICITADO 1
#
# t_s,Rpm
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
# CSV SOLICITADO 2
#
# t_s,Voltaje
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
# GRÁFICAS
# ============================================================================

def plot_curves(
    motor: Motor,
    torque_data: dict[str, np.ndarray],
    voltage_data: dict[str, np.ndarray],
    torque_destination: Path,
    voltage_destination: Path,
    time_rpm_destination: Path,
    time_voltage_destination: Path,
) -> None:

    # ------------------------------------------------------------------------
    # DATOS DE TORQUE
    # ------------------------------------------------------------------------

    torque = (
        torque_data["load_torque_Nm"]
    )

    limited = (
        torque_data["current_limited"]
    )

    normal = ~limited

    limit_torque = (
        torque_data[
            "current_limit_torque_Nm"
        ]
    )

    # ------------------------------------------------------------------------
    # ESTILO GENERAL
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
    # FIGURA 1
    # CARACTERÍSTICAS VS TORQUE
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

        # Región normal.
        ax.plot(
            torque[normal],
            y[normal],
            color=color,
            linestyle="-",
            linewidth=2.3,
        )

        # Región limitada por corriente.
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

        # Punto donde comienza el límite.
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
            linestyle="--",
            linewidth=0.6,
            alpha=0.30,
        )

    # ------------------------------------------------------------------------
    # RPM VS TORQUE
    # ------------------------------------------------------------------------

    draw_curve(
        axes[0, 0],
        torque_data["speed_RPM"],
        speed_color,
        "Speed [RPM]",
        "Rotor Speed vs Load Torque",
    )

    # ------------------------------------------------------------------------
    # CORRIENTE VS TORQUE
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

    # ------------------------------------------------------------------------
    # POTENCIA VS TORQUE
    # ------------------------------------------------------------------------

    draw_curve(
        axes[1, 0],
        torque_data["mechanical_power_W"],
        power_color,
        "Mechanical Power [W]",
        "Mechanical Power vs Load Torque",
    )

    # ------------------------------------------------------------------------
    # EFICIENCIA VS TORQUE
    # ------------------------------------------------------------------------

    draw_curve(
        axes[1, 1],
        torque_data["efficiency_percent"],
        efficiency_color,
        "Efficiency [%]",
        "Efficiency vs Load Torque",
    )

    fig1.savefig(
        torque_destination,
        bbox_inches="tight",
        facecolor="white",
    )

    plt.close(fig1)

    # ========================================================================
    # FIGURA 2
    # RPM VS VOLTAJE
    # ========================================================================

    voltage = (
        voltage_data["voltage_V"]
    )

    speed_rpm = (
        voltage_data["speed_RPM"]
    )

    # Obtener un solo punto por nivel de voltaje.

    unique_voltage = np.arange(
        6.0,
        25.0,
        1.0
    )

    unique_rpm = []

    for v in unique_voltage:

        indices = np.where(
            np.isclose(
                voltage,
                v
            )
        )[0]

        if len(indices) > 0:

            unique_rpm.append(
                speed_rpm[
                    indices[-1]
                ]
            )

    unique_rpm = np.array(
        unique_rpm
    )

    fig2, ax = plt.subplots(
        figsize=(9.5, 6.5),
        constrained_layout=True,
    )

    ax.plot(
        unique_voltage,
        unique_rpm,
        marker="o",
        color=voltage_color,
        linewidth=2.5,
    )

    ax.set_xlim(
        6,
        24
    )

    ax.set_xticks(
        np.arange(
            6,
            25,
            1
        )
    )

    ax.set_xlabel(
        "Motor Voltage [V]",
        fontsize=18,
    )

    ax.set_ylabel(
        "Speed [RPM]",
        fontsize=18,
    )

    ax.set_title(
        "No-Load Speed vs Motor Voltage",
        fontsize=22,
        fontweight="bold",
    )

    ax.grid(
        True,
        linestyle="--",
        alpha=0.30,
    )

    fig2.savefig(
        voltage_destination,
        bbox_inches="tight",
        facecolor="white",
    )

    plt.close(fig2)

    # ========================================================================
    # FIGURA 3
    #
    # RPM VS TIEMPO
    #
    # ESCALONES
    # ========================================================================

    time_s = (
        voltage_data["time_s"]
    )

    final_time = float(
        time_s[-1]
    )

    fig3, ax = plt.subplots(
        figsize=(10, 6),
        constrained_layout=True,
    )

    ax.plot(
        time_s,
        speed_rpm,
        linewidth=2.5,
        color=speed_color,
    )

    ax.set_xlim(
        0,
        final_time
    )

    ax.set_xticks(
        np.arange(
            0,
            final_time + 1,
            10
        )
    )

    ax.set_xlabel(
        "Time [s]",
        fontsize=16,
    )

    ax.set_ylabel(
        "Speed [RPM]",
        fontsize=16,
    )

    ax.set_title(
        "Speed vs Time - 1 V Step Every 10 s",
        fontsize=20,
        fontweight="bold",
    )

    ax.grid(
        True,
        linestyle="--",
        alpha=0.30,
    )

    fig3.savefig(
        time_rpm_destination,
        bbox_inches="tight",
        facecolor="white",
    )

    plt.close(fig3)

    # ========================================================================
    # FIGURA 4
    #
    # VOLTAJE VS TIEMPO
    #
    # ESCALONES
    # ========================================================================

    fig4, ax = plt.subplots(
        figsize=(10, 6),
        constrained_layout=True,
    )

    ax.plot(
        time_s,
        voltage,
        linewidth=2.5,
        color=voltage_color,
    )

    ax.set_xlim(
        0,
        final_time
    )

    ax.set_ylim(
        5.5,
        24.5
    )

    ax.set_xticks(
        np.arange(
            0,
            final_time + 1,
            10
        )
    )

    ax.set_yticks(
        np.arange(
            6,
            25,
            1
        )
    )

    ax.set_xlabel(
        "Time [s]",
        fontsize=16,
    )

    ax.set_ylabel(
        "Voltage [V]",
        fontsize=16,
    )

    ax.set_title(
        "Voltage vs Time - 1 V Step Every 10 s",
        fontsize=20,
        fontweight="bold",
    )

    ax.grid(
        True,
        linestyle="--",
        alpha=0.30,
    )

    fig4.savefig(
        time_voltage_destination,
        bbox_inches="tight",
        facecolor="white",
    )

    plt.close(fig4)


# ============================================================================
# MAIN
# ============================================================================

def main() -> None:

    motor = Motor()

    # ------------------------------------------------------------------------
    # CARPETA DE RESULTADOS
    # ------------------------------------------------------------------------

    output_directory = (
        Path(__file__).resolve().parent
        / "steady_state_results"
    )

    output_directory.mkdir(
        exist_ok=True
    )

    # ------------------------------------------------------------------------
    # CASO 1
    #
    # CURVAS VS TORQUE
    # ------------------------------------------------------------------------

    torque_data = torque_characteristics(
        motor,
        torque_step=1.0e-5,
    )

    # ------------------------------------------------------------------------
    # CASO 2
    #
    # BARRIDO ESCALONADO
    #
    # 0 - 10 s      -> 6 V
    # 10 - 20 s     -> 7 V
    # 20 - 30 s     -> 8 V
    # ...
    # 170 - 180 s   -> 23 V
    # 180 s         -> 24 V
    #
    # Incremento = 1 V cada 10 s
    #
    # Corriente máxima = 1 A
    # ------------------------------------------------------------------------

    voltage_data = voltage_speed_characteristic(
        motor,
        minimum_voltage=6.0,
        maximum_voltage=24.0,
        voltage_step=1.0,
        seconds_per_step=10.0,
    )

    # ------------------------------------------------------------------------
    # CSV GENERAL DEL TORQUE
    # ------------------------------------------------------------------------

    save_csv(
        torque_data,
        output_directory
        / "torque_characteristics.csv",
    )

    # ------------------------------------------------------------------------
    # CSV GENERAL DEL BARRIDO
    # ------------------------------------------------------------------------

    save_csv(
        voltage_data,
        output_directory
        / "voltage_speed_characteristic.csv",
    )

    # ------------------------------------------------------------------------
    # CSV SOLICITADO 1
    #
    # t_s,Rpm
    # ------------------------------------------------------------------------

    save_time_vs_rpm_csv(
        voltage_data,
        output_directory
        / "tiempo_vs_rpm.csv",
    )

    # ------------------------------------------------------------------------
    # CSV SOLICITADO 2
    #
    # t_s,Voltaje
    # ------------------------------------------------------------------------

    save_time_vs_voltage_csv(
        voltage_data,
        output_directory
        / "tiempo_vs_voltaje.csv",
    )

    # ------------------------------------------------------------------------
    # GRÁFICAS
    # ------------------------------------------------------------------------

    plot_curves(
        motor,
        torque_data,
        voltage_data,

        output_directory
        / "motor_characteristics_vs_torque.png",

        output_directory
        / "speed_vs_voltage.png",

        output_directory
        / "rpm_vs_tiempo.png",

        output_directory
        / "voltaje_vs_tiempo.png",
    )

    # ------------------------------------------------------------------------
    # RESULTADOS EN TERMINAL
    # ------------------------------------------------------------------------

    print()

    print(
        "============================================================"
    )

    print(
        f"Motor: {motor.model}"
    )

    print(
        "============================================================"
    )

    print()

    print(
        "BARRIDO ESCALONADO DE VOLTAJE"
    )

    print(
        "------------------------------------------------------------"
    )

    print(
        "Voltaje inicial : 6 V"
    )

    print(
        "Voltaje final   : 24 V"
    )

    print(
        "Incremento      : 1 V"
    )

    print(
        "Tiempo/escalón  : 10 s"
    )

    print(
        "Tiempo total    : 180 s"
    )

    print(
        "Corriente máx.  : 1 A"
    )

    print()

    print(
        "SECUENCIA:"
    )

    print(
        "------------------------------------------------------------"
    )

    # ------------------------------------------------------------------------
    # Mostrar la secuencia completa.
    # ------------------------------------------------------------------------

    for voltage_level in range(
        6,
        25
    ):

        time_level = (
            voltage_level - 6
        ) * 10

        indices = np.where(
            np.isclose(
                voltage_data["voltage_V"],
                voltage_level
            )
        )[0]

        if len(indices) > 0:

            rpm_level = (
                voltage_data["speed_RPM"][
                    indices[-1]
                ]
            )

            print(
                f"{time_level:3d} s"
                f" -> "
                f"{voltage_level:2d} V"
                f" -> "
                f"{rpm_level:8.2f} RPM"
            )

    print()

    print(
        "ARCHIVOS CSV:"
    )

    print(
        "------------------------------------------------------------"
    )

    print(
        "tiempo_vs_rpm.csv"
    )

    print(
        "    t_s,Rpm"
    )

    print()

    print(
        "tiempo_vs_voltaje.csv"
    )

    print(
        "    t_s,Voltaje"
    )

    print()

    print(
        "FORMA DEL ESCALÓN DE VOLTAJE:"
    )

    print(
        "------------------------------------------------------------"
    )

    print(
        "0   s -> 6 V"
    )

    print(
        "10  s -> 6 V"
    )

    print(
        "10  s -> 7 V"
    )

    print(
        "20  s -> 7 V"
    )

    print(
        "20  s -> 8 V"
    )

    print(
        "..."
    )

    print(
        "170 s -> 23 V"
    )

    print(
        "180 s -> 23 V"
    )

    print(
        "180 s -> 24 V"
    )

    print()

    print(
        "Resultados guardados en:"
    )

    print(
        output_directory
    )

    print()


# ============================================================================
# EJECUTAR
# ============================================================================

if __name__ == "__main__":
    main()