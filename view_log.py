import pandas as pd
import matplotlib.pyplot as plt

# Caminho do arquivo a ser visualizado
# Mostra vel, vel_ref, comando e aceleracao

ARQUIVO_CSV = "logs/20261020_203438/car.csv"
VREF = 1.0

df = pd.read_csv(ARQUIVO_CSV)

t = df["t"].values
x = df["x"].values
y = df["y"].values
v = df["v"].values
vref = [VREF] * len(t)
u = df["u"].values
a = df["a"].values
th = df["th"].values
w = df["w"].values

plt.style.use(
    "seaborn-v0_8-whitegrid"
    if "seaborn-v0_8-whitegrid" in plt.style.available
    else "default"
)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 5), dpi=100, sharex=True)

ax1.plot(t, v, label="v", color="#1f77b4", lw=1.5)
ax1.plot(t, vref, "--", label="vref", color="#d62728", lw=1.5)
ax1.set_ylabel("Velocidade [m/s]")
ax1.set_title("Velocidade")
ax1.legend(fontsize=8)

ax2.plot(t, u, label="u", color="#2ca02c", lw=1.5)
ax2.plot(t, a, label="a", color="#ff7f0e", lw=1.5)
ax2.set_xlabel("Tempo [s]")
ax2.set_ylabel("Comando")
ax2.set_title("Controle e aceleração")
ax2.legend(fontsize=8)

plt.tight_layout()
plt.show()

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 5), dpi=100, sharex=True)

ax1.plot(x, y, label="v", color="#1f77b4", lw=1.5)
# ax1.plot(t, y, "--", label="vref", color="#d62728", lw=1.5)
ax1.set_ylabel("y")
ax1.set_title("x")
ax1.legend(fontsize=8)

ax2.plot(t, th, label="u", color="#2ca02c", lw=1.5)
ax2.set_xlabel("Tempo [s]")
ax2.set_ylabel("theta")
ax2.legend(fontsize=8)

plt.tight_layout()
plt.show()