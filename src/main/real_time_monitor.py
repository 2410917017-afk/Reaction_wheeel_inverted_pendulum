# graficar_serial.py
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

PORT   = 'COM3'  
BAUD   = 115200
N_PTS  = 300      # puntos en pantalla (~30 segundos a 10Hz)

t_data     = deque(maxlen=N_PTS)
theta_data = deque(maxlen=N_PTS)
tdot_data  = deque(maxlen=N_PTS)
pdot_data  = deque(maxlen=N_PTS)
u_data     = deque(maxlen=N_PTS)

ser = serial.Serial(PORT, BAUD, timeout=1)
t0  = None

fig, axes = plt.subplots(3, 1, figsize=(10, 8))
fig.suptitle('Reaction Wheel Pendulum — Tiempo Real', fontsize=13)

def update(frame):
    global t0
    try:
        line = ser.readline().decode().strip()
        vals = line.split('\t')
        if len(vals) < 4:
            return

        if t0 is None:
            import time; t0 = time.time()

        import time
        t_data.append(time.time() - t0)
        theta_data.append(float(vals[0]))
        tdot_data.append(float(vals[1]))
        pdot_data.append(float(vals[2]))
        u_data.append(float(vals[3]))

        for ax in axes:
            ax.cla()

        axes[0].plot(t_data, theta_data, 'b', linewidth=1.5)
        axes[0].axhline(0, color='k', linestyle='--', linewidth=0.8)
        axes[0].set_ylabel('θ [°]')
        axes[0].set_ylim(-20, 20)
        axes[0].grid(True)

        axes[1].plot(t_data, tdot_data, 'r',
                     label='θ̇', linewidth=1.5)
        axes[1].plot(t_data, pdot_data, 'm',
                     label='φ̇', linewidth=1.5)
        axes[1].set_ylabel('Velocidad [°/s]')
        axes[1].legend(loc='upper right')
        axes[1].grid(True)

        axes[2].plot(t_data, u_data, 'k', linewidth=1.5)
        axes[2].axhline(0, color='gray', linestyle='--', linewidth=0.8)
        axes[2].set_ylabel('Control u [V]')
        axes[2].set_xlabel('Tiempo [s]')
        axes[2].set_ylim(-20, 20)
        axes[2].grid(True)

    except:
        pass

ani = animation.FuncAnimation(fig, update, interval=100)
plt.tight_layout()
plt.show()
