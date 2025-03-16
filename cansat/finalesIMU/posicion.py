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
positions = deque(maxlen=10)  # Limitar el número de posiciones almacenadas
quaternions = deque(maxlen=10)

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

# Función para leer datos del serial y organizar en tres arrays: giroscopio, acelerómetro, magnetómetro
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

# Función para calibrar los datos del acelerómetro
def calibrate_accelerometer(accel_data):
    if accel_data.size == 3:
        return A_acc @ (accel_data - b_acc)
    else:
        print("Calibrate accelerometer: Invalid data shape:", accel_data.shape)
        return np.zeros(3)

# Función para calibrar los datos del magnetómetro
def calibrate_magnetometer(mag_data):
    if mag_data.size == 3:
        return A_mag @ (mag_data - b_mag)
    else:
        print("Calibrate magnetometer: Invalid data shape:", mag_data.shape)
        return np.zeros(3)

# Función para ajustar el giroscopio aplicando el sesgo y normalización
def adjust_gyroscope(gyro_data):
    if gyro_data.size == 3:
        adjusted_data = gyro_data - GyroBias
        return adjusted_data  # No normalizamos aquí
    else:
        print("Adjust gyroscope: Invalid data shape:", gyro_data.shape)
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

# Configuración de la gráfica
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
point, = ax.plot([], [], [], 'bo')  # Punto azul para la posición actual
line, = ax.plot([], [], [], 'r--')  # Línea discontinua roja para posiciones anteriores

# Función de inicialización para la animación
def init_line():
    line.set_data([], [])
    line.set_3d_properties([])
    point.set_data([], [])
    point.set_3d_properties([])
    return point, line,

# Función de actualización para la animación
def update(frame):
    gyro, accel, mag = read_serial_data()  # Leer datos y asignarlos a cada array

    # Verificar que los datos leídos no estén vacíos
    if gyro.size != 3 or accel.size != 3 or mag.size != 3:
        print("No valid data received!")
        return point, line,

    # Calibrar los datos
    calibrated_gyro = adjust_gyroscope(gyro)
    calibrated_accel = calibrate_accelerometer(accel)
    smoothed_accel = moving_average(calibrated_accel)  # Aplicar el filtro de promedio móvil
    calibrated_mag = calibrate_magnetometer(mag)

    # Calcular ángulos de Euler a partir de los datos calibrados
    roll, pitch, yaw = calculate_euler_angles(smoothed_accel, calibrated_mag)

    # Calcular cuaterniones
    quaternion = euler_to_quaternion(roll, pitch, yaw)

    # Almacenar la posición
    # Asegúrate de que los valores sean correctos y la posición tenga la forma adecuada
    position = np.array([smoothed_accel[0], smoothed_accel[1], smoothed_accel[2]])

    # Validar la forma de position antes de usarlo
    if position.shape == (3,):
        positions.append(position)
        quaternions.append(quaternion)

        # Actualizar el punto de posición
        point.set_data([position[0]], [position[1]])  # Envolver en listas
        point.set_3d_properties(position[2])
        
        # Actualizar la línea discontinua con las posiciones anteriores
        if len(positions) > 1:
            line.set_data([pos[0] for pos in positions], [pos[1] for pos in positions])
            line.set_3d_properties([pos[2] for pos in positions])
    else:
        print("Invalid position shape:", position.shape)
    
    return point, line,


# Inicia la animación
ani = FuncAnimation(fig, update, init_func=init_line, frames=10, interval=10, blit=True)

plt.show()
