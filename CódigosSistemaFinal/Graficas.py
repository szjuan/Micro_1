# ============================================================
#  Graficas.py — Generación de curvas características
#  Freno Magnético · Motor 57BLDC
#
#  Uso:
#    python Graficas.py                   (abre diálogos de archivo)
#    python Graficas.py datos.csv         (solo serie temporal)
#    python Graficas.py datos.csv pts.csv (serie + puntos de operación)
#
#  Requiere: pandas, matplotlib, numpy
#  Instalar: pip install pandas matplotlib numpy
#
#  Gráficas generadas (según el preinforme):
#
#  De la serie temporal (datos.csv):
#    Fig 1  · V(t) e I(t)
#    Fig 2  · n(t) y ω(t)
#    Fig 3  · TL(t)
#    Fig 4  · Pe(t), Pm(t) y η(t)
#
#  De los puntos de operación (puntos.csv):
#    Prueba 1 (n vs V):
#      Fig 5  · n vs V  (curva velocidad–voltaje)
#    Prueba 2 (caracterización con freno):
#      Fig 5  · I vs TL  (equivalente Fig 4.2a del preinforme)
#      Fig 6  · ω vs TL  (equivalente Fig 4.2b)
#      Fig 7  · Pm vs TL (equivalente Fig 4.3a)
#      Fig 8  · η vs TL  (equivalente Fig 4.3b)
#      Fig 9  · Curvas normalizadas (equivalente Fig 4.4)
# ============================================================

import os
import sys
import tkinter as tk
from tkinter import filedialog

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import pandas as pd

matplotlib.rcParams.update({
    "font.family":       "DejaVu Sans",
    "font.size":         11,
    "axes.titlesize":    13,
    "axes.labelsize":    12,
    "legend.fontsize":   10,
    "figure.dpi":        130,
    "axes.grid":         True,
    "grid.alpha":        0.35,
    "grid.linestyle":    "--",
    "axes.spines.top":   False,
    "axes.spines.right": False,
})

# ── Paleta de colores consistente ────────────────────────────
C_RPM    = "#1F77B4"   # azul   — velocidad
C_OMEGA  = "#17BECF"   # cian   — velocidad angular
C_V      = "#D62728"   # rojo   — voltaje
C_I      = "#FF7F0E"   # naranja — corriente
C_TL     = "#2CA02C"   # verde  — torque
C_PE     = "#9467BD"   # violeta — pot. eléctrica
C_PM     = "#8C564B"   # marrón — pot. mecánica
C_ETA    = "#E377C2"   # rosa   — eficiencia
C_FL     = "#7F7F7F"   # gris   — fuerza galga


# ============================================================
#  CARGA DE ARCHIVOS
# ============================================================
def _pedir_archivo(titulo: str, tipos: list[tuple]) -> str | None:
    root = tk.Tk(); root.withdraw()
    ruta = filedialog.askopenfilename(title=titulo, filetypes=tipos)
    root.destroy()
    return ruta or None


def cargar_datos(ruta: str) -> pd.DataFrame:
    df = pd.read_csv(ruta)
    df.columns = [c.strip() for c in df.columns]
    return df


def detectar_prueba(df: pd.DataFrame) -> str:
    """
    Detecta la prueba activa desde la columna 'prueba'.
    Devuelve 'prueba1' o 'prueba2'.
    """
    if "prueba" in df.columns:
        vals = df["prueba"].dropna().unique()
        if len(vals) > 0:
            return str(vals[0])
    return "prueba2"   # default


# ============================================================
#  UTILIDADES DE GUARDADO
# ============================================================
def _guardar(fig: plt.Figure, directorio: str, nombre: str):
    ruta = os.path.join(directorio, nombre)
    fig.savefig(ruta, bbox_inches="tight", dpi=150)
    print(f"  Guardada: {ruta}")


# ============================================================
#  FIGURAS DE SERIE TEMPORAL  (datos.csv)
# ============================================================
def fig_electricas(df: pd.DataFrame) -> plt.Figure:
    """Fig 1 · V(t) e I(t) — variables eléctricas vs tiempo."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle("Variables Eléctricas vs Tiempo", fontweight="bold")

    t = df["t_s"]

    ax1.plot(t, df["V_V"], color=C_V, linewidth=1.4, label="V (V)")
    ax1.set_ylabel("Voltaje (V)")
    ax1.legend(loc="upper left")

    ax2.plot(t, df["I_A"], color=C_I, linewidth=1.4, label="I (A)")
    ax2.set_ylabel("Corriente (A)")
    ax2.set_xlabel("Tiempo (s)")
    ax2.legend(loc="upper left")

    fig.tight_layout()
    return fig


def fig_velocidad(df: pd.DataFrame) -> plt.Figure:
    """Fig 2 · n(t) y ω(t) — velocidad vs tiempo."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle("Velocidad vs Tiempo", fontweight="bold")

    t = df["t_s"]

    ax1.plot(t, df["n_rpm"], color=C_RPM, linewidth=1.4, label="n (RPM)")
    ax1.set_ylabel("Velocidad (RPM)")
    ax1.legend(loc="upper left")

    ax2.plot(t, df["omega_rads"], color=C_OMEGA, linewidth=1.4, label="ω (rad/s)")
    ax2.set_ylabel("Vel. angular (rad/s)")
    ax2.set_xlabel("Tiempo (s)")
    ax2.legend(loc="upper left")

    fig.tight_layout()
    return fig


def fig_torque_tiempo(df: pd.DataFrame) -> plt.Figure:
    """Fig 3 · TL(t) — torque de carga vs tiempo."""
    fig, ax = plt.subplots(figsize=(10, 4))
    fig.suptitle("Torque de Carga vs Tiempo", fontweight="bold")

    ax.plot(df["t_s"], df["TL_Nm"], color=C_TL, linewidth=1.4, label="$T_L$ (N·m)")
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Torque (N·m)")
    ax.legend()

    fig.tight_layout()
    return fig


def fig_potencia_eficiencia(df: pd.DataFrame) -> plt.Figure:
    """Fig 4 · Pe(t), Pm(t) y η(t) — potencia y eficiencia vs tiempo."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle("Potencia y Eficiencia vs Tiempo", fontweight="bold")

    t = df["t_s"]

    ax1.plot(t, df["Pe_W"], color=C_PE, linewidth=1.4, label="$P_e$ (W)")
    ax1.plot(t, df["Pm_W"], color=C_PM, linewidth=1.4, label="$P_m$ (W)", linestyle="--")
    ax1.set_ylabel("Potencia (W)")
    ax1.legend()

    ax2.plot(t, df["eta_pct"], color=C_ETA, linewidth=1.4, label="η (%)")
    ax2.set_ylabel("Eficiencia (%)")
    ax2.set_xlabel("Tiempo (s)")
    ax2.legend()

    fig.tight_layout()
    return fig


# ============================================================
#  FIGURAS PRUEBA 1  —  Velocidad en función del Voltaje
# ============================================================
def fig_p1_n_vs_V(df: pd.DataFrame) -> plt.Figure:
    """
    Fig 5 (Prueba 1) · n vs V — curva velocidad–voltaje.
    Equivalente a la Fig 4.5(a) del preinforme (estado estacionario).
    """
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_title("Curva Velocidad–Voltaje  (Prueba 1, carga mínima)", fontweight="bold")

    df_s = df.sort_values("V_V")
    ax.plot(df_s["V_V"], df_s["n_rpm"], "o-",
            color=C_RPM, linewidth=1.8, markersize=7, label="$n$ experimental")

    ax.set_xlabel("Voltaje de alimentación $V$ (V)")
    ax.set_ylabel("Velocidad $n$ (RPM)")
    ax.legend()

    # Eje secundario con ω
    ax2 = ax.twinx()
    ax2.set_ylabel("Vel. angular $ω$ (rad/s)", color=C_OMEGA)
    omega = df_s["n_rpm"] * 2 * np.pi / 60
    ax2.plot(df_s["V_V"], omega, "s--",
             color=C_OMEGA, linewidth=1.2, markersize=5, alpha=0.7, label="$ω$ experimental")
    ax2.tick_params(axis="y", labelcolor=C_OMEGA)

    fig.tight_layout()
    return fig


def fig_p1_transitorio(df: pd.DataFrame) -> plt.Figure:
    """
    Fig equivalente a Fig 4.5 del preinforme:
    (a) V(t) y n(t), (b) I(t) — respuesta transitoria ante variaciones de voltaje.
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle(
        "Respuesta Transitoria — Variación de Voltaje  (Prueba 1)",
        fontweight="bold"
    )

    t = df["t_s"]

    ax1_twin = ax1.twinx()
    ax1.plot(t, df["V_V"], color=C_V, linewidth=1.4, label="$V$ (V)")
    ax1_twin.plot(t, df["n_rpm"], color=C_RPM, linewidth=1.4,
                  linestyle="--", label="$n$ (RPM)")
    ax1.set_ylabel("Voltaje (V)", color=C_V)
    ax1_twin.set_ylabel("Velocidad (RPM)", color=C_RPM)
    ax1.tick_params(axis="y", labelcolor=C_V)
    ax1_twin.tick_params(axis="y", labelcolor=C_RPM)

    lines1, labs1 = ax1.get_legend_handles_labels()
    lines2, labs2 = ax1_twin.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labs1 + labs2, loc="upper left", fontsize=9)

    ax2.plot(t, df["I_A"], color=C_I, linewidth=1.4, label="$I$ (A)")
    ax2.set_ylabel("Corriente (A)")
    ax2.set_xlabel("Tiempo (s)")
    ax2.legend()

    fig.tight_layout()
    return fig


# ============================================================
#  FIGURAS PRUEBA 2  —  Caracterización con Freno Magnético
# ============================================================
def fig_p2_I_vs_TL(df: pd.DataFrame) -> plt.Figure:
    """
    Fig equivalente a Fig 4.2(a) del preinforme:
    Corriente vs torque de carga.
    """
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.set_title("Corriente vs Torque de Carga  (Prueba 2)", fontweight="bold")

    df_s = df.sort_values("TL_Nm")
    ax.plot(df_s["TL_Nm"], df_s["I_A"], "o-",
            color=C_I, linewidth=1.8, markersize=7, label="$I$ experimental")

    ax.set_xlabel("Torque de carga $T_L$ (N·m)")
    ax.set_ylabel("Corriente $I$ (A)")
    ax.legend()
    fig.tight_layout()
    return fig


def fig_p2_omega_vs_TL(df: pd.DataFrame) -> plt.Figure:
    """
    Fig equivalente a Fig 4.2(b) del preinforme:
    Velocidad angular vs torque de carga.
    """
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.set_title("Velocidad Angular vs Torque de Carga  (Prueba 2)", fontweight="bold")

    df_s = df.sort_values("TL_Nm")
    ax.plot(df_s["TL_Nm"], df_s["omega_rads"], "o-",
            color=C_OMEGA, linewidth=1.8, markersize=7, label="$ω$ experimental")

    ax_r = ax.twinx()
    ax_r.plot(df_s["TL_Nm"], df_s["n_rpm"], "s--",
              color=C_RPM, linewidth=1.2, markersize=5, alpha=0.7, label="$n$ (RPM)")
    ax_r.set_ylabel("Velocidad $n$ (RPM)", color=C_RPM)
    ax_r.tick_params(axis="y", labelcolor=C_RPM)

    ax.set_xlabel("Torque de carga $T_L$ (N·m)")
    ax.set_ylabel("Vel. angular $ω$ (rad/s)")

    lines1, labs1 = ax.get_legend_handles_labels()
    lines2, labs2 = ax_r.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labs1 + labs2, loc="upper right", fontsize=9)

    fig.tight_layout()
    return fig


def fig_p2_Pm_vs_TL(df: pd.DataFrame) -> plt.Figure:
    """
    Fig equivalente a Fig 4.3(a) del preinforme:
    Potencia mecánica vs torque.
    """
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.set_title("Potencia Mecánica vs Torque de Carga  (Prueba 2)", fontweight="bold")

    df_s = df.sort_values("TL_Nm")
    ax.plot(df_s["TL_Nm"], df_s["Pm_W"], "o-",
            color=C_PM, linewidth=1.8, markersize=7, label="$P_m$ experimental")
    ax.plot(df_s["TL_Nm"], df_s["Pe_W"], "s--",
            color=C_PE, linewidth=1.4, markersize=5, alpha=0.8, label="$P_e$ (entrada)")

    ax.set_xlabel("Torque de carga $T_L$ (N·m)")
    ax.set_ylabel("Potencia (W)")
    ax.legend()
    fig.tight_layout()
    return fig


def fig_p2_eta_vs_TL(df: pd.DataFrame) -> plt.Figure:
    """
    Fig equivalente a Fig 4.3(b) del preinforme:
    Eficiencia vs torque.
    """
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.set_title("Eficiencia vs Torque de Carga  (Prueba 2)", fontweight="bold")

    df_s = df.sort_values("TL_Nm")
    ax.plot(df_s["TL_Nm"], df_s["eta_pct"], "o-",
            color=C_ETA, linewidth=1.8, markersize=7, label="η experimental")

    ax.set_xlabel("Torque de carga $T_L$ (N·m)")
    ax.set_ylabel("Eficiencia η (%)")
    ax.set_ylim(bottom=0)
    ax.legend()
    fig.tight_layout()
    return fig


def fig_p2_normalizada(df: pd.DataFrame) -> plt.Figure:
    """
    Fig equivalente a Fig 4.4 del preinforme:
    Curvas normalizadas (n, I, Pm, η) vs torque de carga.
    Cada variable se normaliza a su máximo dentro del rango analizado.
    """
    fig, ax = plt.subplots(figsize=(9, 6))
    ax.set_title(
        "Curvas Características Normalizadas  (Prueba 2)",
        fontweight="bold"
    )

    df_s = df.sort_values("TL_Nm").copy()
    x = df_s["TL_Nm"]

    def norm(series):
        mx = series.max()
        return series / mx if mx != 0 else series

    series_map = {
        "$n$ (RPM)":   (df_s["n_rpm"],  C_RPM,  "-",  "o"),
        "$I$ (A)":     (df_s["I_A"],    C_I,    "-",  "s"),
        "$P_m$ (W)":   (df_s["Pm_W"],   C_PM,   "--", "^"),
        "η (%)":       (df_s["eta_pct"],C_ETA,  ":",  "D"),
    }

    for label, (serie, color, ls, marker) in series_map.items():
        y = norm(serie)
        ax.plot(x, y, linestyle=ls, marker=marker,
                color=color, linewidth=1.6, markersize=6, label=label)

    ax.set_xlabel("Torque de carga $T_L$ (N·m)")
    ax.set_ylabel("Valor normalizado (u.a.)")
    ax.set_ylim(0, 1.12)
    ax.yaxis.set_major_formatter(ticker.PercentFormatter(xmax=1, decimals=0))
    ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5))
    fig.tight_layout()
    return fig


def fig_p2_transitorio(df: pd.DataFrame) -> plt.Figure:
    """
    Fig equivalente a Fig 4.6 del preinforme:
    (a) TL(t) y n(t), (b) I(t) — respuesta transitoria ante variaciones de carga.
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle(
        "Respuesta Transitoria — Variación de Carga  (Prueba 2)",
        fontweight="bold"
    )

    t = df["t_s"]

    ax1_twin = ax1.twinx()
    ax1.plot(t, df["TL_Nm"], color=C_TL, linewidth=1.4, label="$T_L$ (N·m)")
    ax1_twin.plot(t, df["n_rpm"], color=C_RPM, linewidth=1.4,
                  linestyle="--", label="$n$ (RPM)")
    ax1.set_ylabel("Torque (N·m)", color=C_TL)
    ax1_twin.set_ylabel("Velocidad (RPM)", color=C_RPM)
    ax1.tick_params(axis="y", labelcolor=C_TL)
    ax1_twin.tick_params(axis="y", labelcolor=C_RPM)

    lines1, labs1 = ax1.get_legend_handles_labels()
    lines2, labs2 = ax1_twin.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labs1 + labs2, loc="upper right", fontsize=9)

    ax2.plot(t, df["I_A"], color=C_I, linewidth=1.4, label="$I$ (A)")
    ax2.set_ylabel("Corriente (A)")
    ax2.set_xlabel("Tiempo (s)")
    ax2.legend()

    fig.tight_layout()
    return fig


# ============================================================
#  GENERAR TODAS LAS FIGURAS
# ============================================================
def generar_figuras(
    df_datos: pd.DataFrame | None,
    df_puntos: pd.DataFrame | None,
    directorio: str,
):
    """
    Genera y guarda todas las figuras según los datos disponibles.
    """
    print(f"\nGuardando figuras en: {directorio}\n")
    os.makedirs(directorio, exist_ok=True)

    # ── Detectar tipo de prueba ────────────────────────────────
    prueba = "prueba2"
    if df_puntos is not None and not df_puntos.empty:
        prueba = detectar_prueba(df_puntos)
    elif df_datos is not None and not df_datos.empty:
        prueba = detectar_prueba(df_datos)

    print(f"Prueba detectada: {prueba}\n")

    # ── Figuras de serie temporal ──────────────────────────────
    if df_datos is not None and not df_datos.empty:
        d = df_datos

        _guardar(fig_electricas(d),           directorio, "fig1_V_I_vs_t.png")
        plt.close()
        _guardar(fig_velocidad(d),             directorio, "fig2_n_omega_vs_t.png")
        plt.close()
        _guardar(fig_torque_tiempo(d),         directorio, "fig3_TL_vs_t.png")
        plt.close()
        _guardar(fig_potencia_eficiencia(d),   directorio, "fig4_Pe_Pm_eta_vs_t.png")
        plt.close()

        if prueba == "prueba1":
            _guardar(fig_p1_transitorio(d), directorio, "fig5_transitorio_V.png")
            plt.close()
        else:
            _guardar(fig_p2_transitorio(d), directorio, "fig5_transitorio_TL.png")
            plt.close()

    # ── Figuras de puntos de operación ────────────────────────
    if df_puntos is not None and not df_puntos.empty:
        p = df_puntos

        if prueba == "prueba1":
            _guardar(fig_p1_n_vs_V(p),      directorio, "fig6_n_vs_V.png")
            plt.close()
        else:
            _guardar(fig_p2_I_vs_TL(p),         directorio, "fig6_I_vs_TL.png")
            plt.close()
            _guardar(fig_p2_omega_vs_TL(p),      directorio, "fig7_omega_vs_TL.png")
            plt.close()
            _guardar(fig_p2_Pm_vs_TL(p),         directorio, "fig8_Pm_vs_TL.png")
            plt.close()
            _guardar(fig_p2_eta_vs_TL(p),        directorio, "fig9_eta_vs_TL.png")
            plt.close()
            _guardar(fig_p2_normalizada(p),       directorio, "fig10_normalizada.png")
            plt.close()

    print("\n¡Listo! Todas las figuras generadas.")


# ============================================================
#  MAIN
# ============================================================
def main():
    tipos_csv = [("Archivos CSV", "*.csv"), ("Todos", "*.*")]

    # Argumentos por línea de comandos
    ruta_datos  = sys.argv[1] if len(sys.argv) > 1 else None
    ruta_puntos = sys.argv[2] if len(sys.argv) > 2 else None

    # Si no se pasaron argumentos, preguntar con diálogos
    if ruta_datos is None:
        print("Selecciona el archivo de DATOS (serie temporal):")
        ruta_datos = _pedir_archivo(
            "Seleccionar archivo de datos (serie temporal)", tipos_csv
        )

    if ruta_datos is None:
        print("No se seleccionó archivo de datos. Saliendo.")
        return

    df_datos = cargar_datos(ruta_datos)
    print(f"Datos cargados: {len(df_datos)} filas · {ruta_datos}")

    # Archivo de puntos de operación (opcional)
    df_puntos = None
    if ruta_puntos is None:
        print("\nSelecciona el archivo de PUNTOS DE OPERACIÓN (opcional — cierra para omitir):")
        ruta_puntos = _pedir_archivo(
            "Seleccionar archivo de puntos de operación (opcional)", tipos_csv
        )

    if ruta_puntos:
        df_puntos = cargar_datos(ruta_puntos)
        print(f"Puntos cargados: {len(df_puntos)} filas · {ruta_puntos}")
    else:
        print("Sin archivo de puntos — solo se generarán figuras de serie temporal.")

    # Carpeta de destino: misma carpeta que el archivo de datos
    directorio_salida = os.path.join(
        os.path.dirname(os.path.abspath(ruta_datos)),
        "Figuras"
    )

    generar_figuras(df_datos, df_puntos, directorio_salida)

    # Mostrar las figuras en pantalla (opcional)
    plt.show()


if __name__ == "__main__":
    main()
