# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ============================================================
# CONFIGURAÇÕES DE VISUALIZAÇÃO DO LOG
# ============================================================
ARQUIVO_CSV = "logs/20261022_162914/car.csv"  # Altere para o CSV que deseja analisar
VREF = 1.0                                    # Referência de velocidade [m/s]
YREF = -9.0                                   # Referência de posição lateral Y [m]
THETA_REF_DEG = 180.0                         # Referência de orientação [graus] (pi rad = 180°)

# Carrega os dados
df = pd.read_csv(ARQUIVO_CSV)

t = df["t"].values
x = df["x"].values
y = df["y"].values
v = df["v"].values
u = df["u"].values
a = df["a"].values
th = df["th"].values
w = df["w"].values

# Cálculos auxiliares para diagnóstico do controle lateral
erro_y_cm = (y - YREF) * 100.0  # Erro lateral em centímetros

# Direção medida pelo quaternion convertida para graus
th_deg = np.rad2deg(th)
th_deg[0] = th_deg[1] if len(th_deg) > 1 else th_deg[0]

# Direção real do movimento calculada pela derivada da posição
dx = np.gradient(x)
dy = np.gradient(y)
heading_real_deg = np.rad2deg(np.arctan2(dy, dx)) + 180.0

# Estimativa do ângulo de esterçamento efetivo pelo modelo cinemático: delta = atan(L * w / v)
L = 0.302
if "st" in df.columns:
    steer_deg = np.rad2deg(df["st"].values)
    label_steer = "Comando steer salvo (st) [°]"
else:
    v_safe = np.where(np.abs(v) > 0.1, v, 0.1)
    steer_deg = np.rad2deg(np.arctan(L * w / v_safe))
    label_steer = "Esterçamento cinemático est. atan(L*w/v) [°]"

# Métricas de desempenho lateral
mae_y_cm = np.mean(np.abs(erro_y_cm))
max_y_cm = np.max(np.abs(erro_y_cm))
erro_final_cm = erro_y_cm[-1]

plt.style.use(
    "seaborn-v0_8-whitegrid"
    if "seaborn-v0_8-whitegrid" in plt.style.available
    else "default"
)

# ============================================================
# Figura 1: Diagnóstico completo do controle lateral
# ============================================================
fig1, axes = plt.subplots(4, 1, figsize=(9, 9), dpi=100)

# Trajetória no plano xy
axes[0].plot(x, y, label="Trajetória", color="#1f77b4", lw=2.0)
axes[0].axhline(YREF, color="#d62728", linestyle="--", lw=1.5, label=f"y_ref = {YREF:.2f} m")
axes[0].plot(x[0], y[0], "go", markersize=7, label="Início")
axes[0].plot(x[-1], y[-1], "rs", markersize=7, label="Fim")
axes[0].set_xlabel("x [m]")
axes[0].set_ylabel("y [m]")
axes[0].set_title(
    f"Visualização da trajetória",
    fontweight="bold",
)
axes[0].legend(loc="best", fontsize=8, frameon=True)

# Erro lateral
axes[1].plot(t, erro_y_cm, label="Erro lateral (y - y_ref)", color="#ff7f0e", lw=1.8)
axes[1].axhline(0.0, color="#2ca02c", linestyle="--", lw=1.2)
axes[1].set_ylabel("Erro y [cm]")
axes[1].set_title("Crosstrack error")
axes[1].legend(loc="upper right", fontsize=8, frameon=True)

# angulo de orientação
axes[2].plot(t, th_deg, label=r"Yaw ($\theta$) [°]", color="#9467bd", lw=1.2, alpha=0.8)
axes[2].plot(t, heading_real_deg, label="Direção real da trajetória filtrada atan2(dy,dx) [°]", color="#2ca02c", lw=2.0)
axes[2].axhline(THETA_REF_DEG, color="#d62728", linestyle=":", lw=1.2, label=f"Ref ({THETA_REF_DEG:.0f}°)")
axes[2].set_ylabel("Ângulo [graus]")
axes[2].set_title("Orientação (heading): yaw vs trajetória real")
axes[2].legend(loc="upper right", fontsize=8, frameon=True)

# velocidade angular w e esterçamento delta
axes[3].plot(t, steer_deg, label=label_steer, color="#d62728", lw=1.5)
axes[3].plot(t, np.rad2deg(w), label=r"Vel. angular ($w$) [°/s]", color="#1f77b4", lw=1.2, alpha=0.7)
axes[3].axhline(0.0, color="black", linestyle="--", lw=1.0)
axes[3].set_xlabel("Tempo [s]")
axes[3].set_ylabel("Graus / [°/s]")
axes[3].set_title("Ação direcional e yaw rate")
axes[3].legend(loc="upper right", fontsize=8, frameon=True)

plt.tight_layout()

# ============================================================
# Figura 2: Diagnóstico de controle longitudinal
# ============================================================
fig2, (ax_v, ax_u) = plt.subplots(2, 1, figsize=(8, 5), dpi=100, sharex=True)

ax_v.plot(t, v, label="Velocidade (v)", color="#1f77b4", lw=1.8)
ax_v.axhline(VREF, linestyle="--", label=f"Referência ({VREF} m/s)", color="#d62728", lw=1.5)
ax_v.set_ylabel("Velocidade [m/s]")
ax_v.set_title("Controle longitudinal - Velocidade", fontweight="bold")
ax_v.legend(loc="lower right", fontsize=8, frameon=True)

ax_u.plot(t, u, label="Comando (u)", color="#2ca02c", lw=1.5)
ax_u.plot(t, a, label="Aceleração (a) [m/s²]", color="#ff7f0e", lw=1.5, alpha=0.8)
ax_u.set_xlabel("Tempo [s]")
ax_u.set_ylabel("Comando / Acel.")
ax_u.set_title("Esforço de controle longitudinal e aceleração")
ax_u.legend(loc="upper right", fontsize=8, frameon=True)

plt.tight_layout()
plt.show()