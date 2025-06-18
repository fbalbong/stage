import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation

# === Leer CSV ===
df = pd.read_csv("vuelo_datos.csv")
t = df['time']

# === Gráficas 2D Kalman vs Lighthouse ===
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(t, df['x_kalman'], label='x Kalman')
plt.plot(t, df['x_lh'], ':', label='x Lighthouse')
plt.ylabel('X (m)')
plt.grid()
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(t, df['y_kalman'], label='y Kalman')
plt.plot(t, df['y_lh'], ':', label='y Lighthouse')
plt.ylabel('Y (m)')
plt.grid()
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(t, df['z_kalman'], label='z Kalman')
plt.plot(t, df['z_lh'], ':', label='z Lighthouse')
plt.xlabel('Tiempo (s)')
plt.ylabel('Z (m)')
plt.grid()
plt.legend()

plt.suptitle('Comparación Kalman vs Lighthouse por eje')
plt.tight_layout()
plt.show()

# === Gráfico 3D estático ===
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
ax.plot(df['x_kalman'], df['y_kalman'], df['z_kalman'], label='Kalman')
ax.plot(df['x_lh'], df['y_lh'], df['z_lh'], ':', label='Lighthouse')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Trayectoria 3D Kalman vs Lighthouse')
ax.legend()
plt.tight_layout()
plt.show()

# === Animación 3D ===
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([min(df['x_kalman']), max(df['x_kalman'])])
ax.set_ylim([min(df['y_kalman']), max(df['y_kalman'])])
ax.set_zlim([min(df['z_kalman']), max(df['z_kalman'])])
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Animación 3D Kalman vs Lighthouse')

line_kalman, = ax.plot([], [], [], label='Kalman', color='blue')
line_lh, = ax.plot([], [], [], label='Lighthouse', color='orange', linestyle='dashed')
ax.legend()

def init():
    line_kalman.set_data([], [])
    line_kalman.set_3d_properties([])
    line_lh.set_data([], [])
    line_lh.set_3d_properties([])
    return line_kalman, line_lh

def update(frame):
    line_kalman.set_data(df['x_kalman'][:frame], df['y_kalman'][:frame])
    line_kalman.set_3d_properties(df['z_kalman'][:frame])
    line_lh.set_data(df['x_lh'][:frame], df['y_lh'][:frame])
    line_lh.set_3d_properties(df['z_lh'][:frame])
    return line_kalman, line_lh

frames = min(len(df), len(df))
ani = animation.FuncAnimation(fig, update, init_func=init, frames=frames, interval=100, blit=True)

plt.tight_layout()
plt.show()
