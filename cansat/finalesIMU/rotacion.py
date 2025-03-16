import numpy as np
import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from collections import deque

# Configura el puerto serial
ser = serial.Serial('COM3', 115200)  # Cambia 'COM4' por tu puerto serial

# Define los parámetros de calibración del magnetómetro
A_mag = np.array([[0.844300, 0.001799, -0.004043],
                  [0.001799, 0.877542, 0.053194],
                  [-0.004043, 0.053194, 0.908574]])
b_mag = np.array([-23.421733, 17.789619, -14.002814])

# Define los parámetros de calibración del acelerómetro
A_acc = np.array([[0.193299, 0.000763, 0.000534],
                  [0.000763, 0.195531, -0.003699],
                  [0.000534, -0.003699, 0.192057]])
b_acc = np.array([1.466454, -1.854705, 0.859415])

# Parámetros de sesgo del giroscopio
GyroBias = np.array([1.21, 1.18, 1.03138])  # Convertir a rad/s

# Inicializa la posición de referencia
reference_position = np.zeros(3)

# Almacena las posiciones y cuaterniones
positions = deque(maxlen=5)  # Limitar el número de posiciones almacenadas
quaternions = deque(maxlen=5)

# Parámetros del filtro
window_size = 3  # Tamaño de la ventana para el promedio móvil
accel_history = deque(maxlen=window_size)

def moving_average(data):
    if len(accel_history) < window_size:
        accel_history.append(data)
        return np.mean(accel_history, axis=0)
    else:
        accel_history.append(data)
        return np.mean(accel_history, axis=0)

# Función para leer datos del serial
def read_serial_data():
    line = ser.readline().decode('utf-8').strip()
    print(f"Raw line: {line}")  # Para ver qué se está recibiendo
    try:
        data = np.array([float(val) for val in line.split(',')])
        if data.size == 9:
            gyro_data = data[0:3]   # Giroscopio
            accel_data = data[3:6]  # Acelerómetro
            mag_data = data[6:9]    # Magnetómetro
            return gyro_data, accel_data, mag_data
        else:
            print("Invalid data length:", data.size)
            return np.array([]), np.array([]), np.array([])
    except ValueError:
        print("Error parsing data: ", line)
        return np.array([]), np.array([]), np.array([])

# Funciones de calibración
def calibrate_accelerometer(accel_data):
    if accel_data.size == 3:
        return A_acc @ (accel_data - b_acc)
    else:
        return np.zeros(3)

def calibrate_magnetometer(mag_data):
    if mag_data.size == 3:
        return A_mag @ (mag_data - b_mag)
    else:
        return np.zeros(3)

def adjust_gyroscope(gyro_data):
    if gyro_data.size == 3:
        return gyro_data - GyroBias
    else:
        return np.zeros(3)

# Función para calcular cuaterniones a partir de los ángulos de Euler
def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return np.array([qx, qy, qz, qw])

# Función para calcular ángulos de Euler
def calculate_euler_angles(accel, mag):
    roll = np.arctan2(accel[1], accel[2])
    pitch = np.arctan2(-accel[0], np.sqrt(accel[1] ** 2 + accel[2] ** 2))
    yaw = np.arctan2(mag[1], mag[0])
    return roll, pitch, yaw

# Función para convertir cuaterniones a matriz de rotación
def quaternion_to_rotation_matrix(q):
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)]
    ])

# Configuración de la gráfica
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

# Define el cubo
def draw_cube(ax, position, orientation):
    r = 0.15  # Tamaño del cubo
    # Define los 8 vértices del cubo
    vertices = np.array([[ r,  r,  r],
                         [ r,  r, -r],
                         [ r, -r,  r],
                         [ r, -r, -r],
                         [-r,  r,  r],
                         [-r,  r, -r],
                         [-r, -r,  r],
                         [-r, -r, -r]])
    # Aplica la rotación al cubo
    rotation_matrix = quaternion_to_rotation_matrix(orientation)
    rotated_vertices = vertices @ rotation_matrix.T + position  # Aplica la rotación y la traslación
    
    # Define las aristas del cubo
    edges = [
        [0, 1], [0, 2], [0, 4],
        [1, 3], [1, 5],
        [2, 3], [2, 6],
        [3, 7],
        [4, 5], [4, 6],
        [5, 7],
        [6, 7]
    ]
    
    colors = ['r', 'g', 'b', 'c', 'm', 'y']  # Colores para las aristas
    artists = []
    for i, edge in enumerate(edges):
        line, = ax.plot3D(*zip(rotated_vertices[edge[0]], rotated_vertices[edge[1]]), color=colors[i % len(colors)], linewidth=2)
        artists.append(line)
    return artists

# Función de actualización para la animación
def update(frame):
    gyro, accel, mag = read_serial_data()  # Leer datos y asignarlos a cada array

    if gyro.size != 3 or accel.size != 3 or mag.size != 3:
        print("No valid data received!")
        return []

    # Calibrar los datos
    calibrated_gyro = adjust_gyroscope(gyro)
    calibrated_accel = calibrate_accelerometer(accel)
    smoothed_accel = moving_average(calibrated_accel)
    calibrated_mag = calibrate_magnetometer(mag)

    # Calcular ángulos de Euler y cuaterniones
    roll, pitch, yaw = calculate_euler_angles(smoothed_accel, calibrated_mag)
    quaternion = euler_to_quaternion(roll, pitch, yaw)

    # Almacenar la posición
    position = np.array([smoothed_accel[0], smoothed_accel[1], smoothed_accel[2]])

    # Actualizar la gráfica del cubo
    ax.cla()  # Limpiar la gráfica para volver a dibujar
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    return draw_cube(ax, position, quaternion)

# Inicializa la animación
ani = FuncAnimation(fig, update, init_func=lambda: draw_cube(ax, np.zeros(3), np.array([1, 0, 0, 0])), frames=10, interval=10, blit=True)

plt.show()
