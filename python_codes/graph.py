import pandas as pd
import matplotlib.pyplot as plt

# Leer el archivo CSV
# df = pd.read_csv("comparacion_sin_ligthouse.csv")
df = pd.read_csv("sin_lighthouse_deck.csv")

# Asumimos que cada fila corresponde a un instante temporal uniforme
t = range(len(df))

# Extraer datos Kalman y Lighthouse
kalman_x = df['stateEstimate.x_y']
kalman_y = df['stateEstimate.y_y']
kalman_z = df['stateEstimate.z_y']

lighthouse_x = df['lighthouse.x_y']
lighthouse_y = df['lighthouse.y_y']
lighthouse_z = df['lighthouse.z_y']

# Crear gráficas
plt.figure(figsize=(12, 8))

# Eje X
plt.subplot(3, 1, 1)
plt.plot(t, kalman_x, label='x estimé (Kalman)')
plt.plot(t, lighthouse_x, ':', label='x lighthouse')
plt.ylabel('X (m)')
plt.grid()
plt.legend()

# Eje Y
plt.subplot(3, 1, 2)
plt.plot(t, kalman_y, label='y estimé (Kalman)')
plt.plot(t, lighthouse_y, ':', label='y lighthouse')
plt.ylabel('Y (m)')
plt.grid()
plt.legend()

# Eje Z
plt.subplot(3, 1, 3)
plt.plot(t, kalman_z, label='z estimé (Kalman)')
plt.plot(t, lighthouse_z, ':', label='z lighthouse')
plt.xlabel('Tiempo (arbitrary units)')
plt.ylabel('Z (m)')
plt.grid()
plt.legend()

plt.suptitle('Comparación Kalman vs Lighthouse')
plt.tight_layout()
plt.show()
