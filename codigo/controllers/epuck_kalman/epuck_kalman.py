from controller import Robot
import csv

MODO_DATOS: str = 'kalman'
""" Define los datos que utilizara el sistema de navegación reactiva """

# =============================
# -- Umbrales con histéresis --
# =============================
UMBRAL_ENTRAR_EVASION: float = 0.045  # 
""" Reacciona tan pronto vea algo a 4.5 cm """

UMBRAL_SALIR_EVASION: float = 0.048   
""" Vuelve a avanzar cuando el camino esté casi despejado (4.8 cm) """

UMBRAL_LATERAL_SEGURO: int = 200 
""" Valor crudo de los sensores laterales por debajo del cual consideramos que no hay peligro (ajustable) """

VAR_RUIDO: int = 100 
""" Valor crudo por debajo del cual consideramos que no hay nada (ajustable) """

LATERAL_DEADBAND_RAW: float = 35.0
""" Si la diferencia entre los sensores laterales es menor a este valor, consideramos que el obstáculo está relativamente centrado y mantenemos la dirección de giro previa para evitar oscilaciones (ajustable). """

# ===========================
# -- Constantes del E-puck --
# ===========================
robot = Robot()
TIME_STEP: int = int(robot.getBasicTimeStep())
""" 
Tiempo de muestreo del controlador en milisegundos (en este caso 32 ms para el e-puck) 
Se define como entero porque el método robot.step() espera un entero.
"""
WHEEL_RADIUS: float = 0.0205  
""" Radio de la rueda en metros (20.5 mm) """

MAX_SPEED: float = 6.28       
""" Velocidad máxima de los motores en radianes por segundo (6.28 rad/s = 1 vuelta/s) """

# ====================================
# -- Variables del Filtro de Kalman --
# ====================================
d_k_est: float = 0.05       
""" 
Estimación inicial de la distancia al obstáculo: comenzamos con el valor máximo que el sensor puede medir (5 cm).
""" 

P_k: float = 0.1            
""" 
Covarianza inicial del error: comenzamos con una incertidumbre moderada, 
ya que la estimación inicial es solo una suposición.
Ajustable según el nivel de confianza que queramos darle al filtro al inicio. 
"""

R: float = 0.02             
"""
Varianza del ruido del sensor: representa la incertidumbre de las mediciones del sensor.
Un valor más bajo indica que confiamos mucho en el sensor, 
mientras que un valor más alto indica que el sensor es ruidoso y el filtro confiará más en la predicción del modelo de movimiento.
"""

Q: float = 0.06            
"""
Ruido del proceso: representa la incertidumbre en el modelo de movimiento (avance estimado por los encoders).
Un valor más bajo indica que confiamos mucho en el modelo de movimiento,
mientras que un valor más alto indica que el modelo de movimiento es incierto y el filtro confiará más en las mediciones del sensor.
"""

# Variables para cálculo de avance con encoders
prev_left_enc: float = 0.0
"""
Valor del encoder izquierdo en el paso anterior, usado para calcular el avance del robot entre pasos.
"""

prev_right_enc: float = 0.0
"""
Valor del encoder derecho en el paso anterior, usado para calcular el avance del robot entre pasos.
"""

historial_frontal: list[float] = [0.0, 0.0, 0.0] # Variables filtro simple (3 muestras)
"""
Historial de las últimas 3 lecturas crudas del sensor frontal para aplicar un filtro de promedio móvil simple.
"""

# Estado de navegación para estabilizar decisiones entre pasos.
modo_evasion: bool = False
ultima_direccion_giro: int = 1  # 1: gira derecha, -1: gira izquierda


# ==========================
# -- Funciones Auxiliares --
# ==========================

def init_motores():
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.2 * MAX_SPEED)
    right_motor.setVelocity(0.2 * MAX_SPEED)
    return left_motor, right_motor

def init_sensoresdistancia():
    ps0 = robot.getDevice('ps0')
    ps7 = robot.getDevice('ps7')
    ps0.enable(TIME_STEP)
    ps7.enable(TIME_STEP)

    ps2 = robot.getDevice('ps2') # Derecho
    ps5 = robot.getDevice('ps5') # Izquierdo
    ps2.enable(TIME_STEP)
    ps5.enable(TIME_STEP)

    return ps0, ps7, ps2, ps5

def init_encoders():
    left_encoder = robot.getDevice('left wheel sensor')
    right_encoder = robot.getDevice('right wheel sensor')
    left_encoder.enable(TIME_STEP)
    right_encoder.enable(TIME_STEP)
    return left_encoder, right_encoder

def convertir_sensor_a_metros(valor_crudo):
    """
    Los sensores del e-puck leen de 0 (lejos) a 4095 (cerca).
    El filtro de Kalman necesita que esto se cruce con los encoders (metros).
    Esta es una aproximación lineal simple (max 5 cm).
    """
    if valor_crudo < VAR_RUIDO: # Esto es para evitar ruidos cuando no hay nada cerca, ya que el sensor puede dar valores bajos incluso sin obstáculos.
        return 0.05 # Si no hay nada, asumimos max distancia (5cm)
    distancia = 0.05 * (1.0 - (valor_crudo / 4095.0))
    return distancia

def apply_kalman_filter(d_k_est, P_k, z_k, avance_s, Q, R):
    """
    Aplica una iteración del filtro de Kalman para estimar la distancia al obstáculo.
    
    Parámetros:
        d_k_est: Estimación anterior de la distancia (metros)
        P_k: Covarianza anterior del error
        z_k: Medida del sensor filtrada (metros)
        avance_s: Movimiento del robot desde el paso anterior (metros)
        Q: Ruido del proceso (incertidumbre de los encoders)
        R: Ruido del sensor
    
    Retorna:
        (d_k_est_new, P_k_new): Distancia estimada y covarianza actualizadas
    """
    # 1. Etapa de Predicción
    d_k_prediccion = d_k_est - avance_s
    P_k_prediccion = P_k + Q
    
    # 2. Ganancia de Kalman
    K_k = P_k_prediccion / (P_k_prediccion + R)
    
    # 3. Etapa de Corrección
    d_k_est_new = d_k_prediccion + K_k * (z_k - d_k_prediccion)
    
    # 4. Actualización de la Covarianza
    P_k_new = (1 - K_k) * P_k_prediccion
    
    return d_k_est_new, P_k_new

def compute_motor_speeds(front_distance_meters, left_sensor_raw, right_sensor_raw, max_speed, modo_evasion_actual, ultima_direccion_actual):
    """
    Lógica de navegación reactiva para el e-puck,
    Decide la velocidad de los motores basándose en la distancia frontal
    y los sensores laterales (ps6 y ps1).
    """

    # Velocidad base de crucero (la mitad de la máxima)
    base_speed = 0.5 * max_speed

    # Máquina de estados con histéresis:
    # Entra a evasión cuando la distancia cae por debajo del umbral de entrada.
    if (not modo_evasion_actual) and (front_distance_meters <= UMBRAL_ENTRAR_EVASION):
        modo_evasion_actual = True
        
    # Intenta salir de evasión cuando el frente se despeje.
    elif modo_evasion_actual and (front_distance_meters >= UMBRAL_SALIR_EVASION):
        if left_sensor_raw < UMBRAL_LATERAL_SEGURO and right_sensor_raw < UMBRAL_LATERAL_SEGURO:
            modo_evasion_actual = False

    if not modo_evasion_actual:
        # El camino está libre, avanzamos en línea recta.
        left_speed = base_speed
        right_speed = base_speed
    else:
        # En los sensores del e-puck, un valor crudo mayor significa que el objeto está mas cerca.
        delta_lateral = left_sensor_raw - right_sensor_raw
        
        if abs(delta_lateral) <= LATERAL_DEADBAND_RAW:
            direccion_giro = ultima_direccion_actual
        elif delta_lateral > 0:
            # El obstáculo está más próximo por la izquierda, gira a la derecha.
            direccion_giro = 1
        else:
            # El obstáculo está más próximo por la derecha, gira a la izquierda.
            direccion_giro = -1

        # Aplicamos las velocidades según la dirección de giro.
        if direccion_giro == 1:
            left_speed = base_speed
            right_speed = -base_speed
        else:
            left_speed = -base_speed
            right_speed = base_speed

        # Guardamos la dirección para mantener el giro si el delta lateral se vuelve pequeño
        ultima_direccion_actual = direccion_giro

    return left_speed, right_speed, modo_evasion_actual, ultima_direccion_actual

# -- Información de Muestreo --
freq = 1000 / TIME_STEP
print(f"Tiempo de muestreo (Ts): {TIME_STEP} ms")
print(f"Frecuencia de muestreo (fs): {freq} Hz")

# --- Inicialización de Motores ---
left_motor, right_motor = init_motores()

# --- Inicialización de Sensores de Distancia ---
ps0, ps7, ps2, ps5 = init_sensoresdistancia()

# --- Inicialización de Encoders ---
left_encoder, right_encoder = init_encoders()

# --- Almacenamos las señales en un .csv ---
archivo_csv = open('../../datos_sensores.csv', 'w', newline='')
writer = csv.writer(archivo_csv)
writer.writerow(['Tiempo', 'Sensor_Crudo', 'Sensor_Filtrado', 'Sensor_Metros', 'Avance_Encoder', 'Kalman_Distancia'])

# =====================
# -- Bucle Principal --
# =====================
while robot.step(TIME_STEP) != -1:
    tiempo_actual = robot.getTime()
    
    current_left_enc = left_encoder.getValue()
    current_right_enc = right_encoder.getValue()
    
    raw_ps0 = ps0.getValue()
    raw_ps7 = ps7.getValue()
    
    # Tomamos el valor MÁXIMO en lugar del promedio.
    # Si cualquier sensor ve peligro, reaccionamos a ese.
    valor_peligro_maximo = max(raw_ps0, raw_ps7)
    
    # --- Filtro Simple (Promedio Móvil) ---
    historial_frontal.pop(0)
    historial_frontal.append(valor_peligro_maximo) # Usamos el máximo aquí
    sensor_filtrado_crudo = sum(historial_frontal) / len(historial_frontal)
    
    # Convertimos la lectura filtrada a metros para el Kalman
    z_k = convertir_sensor_a_metros(sensor_filtrado_crudo)
    
    # --- Estimación del Avance mediante Encoders (s = r * theta) ---
    delta_theta_left = current_left_enc - prev_left_enc
    delta_theta_right = current_right_enc - prev_right_enc
    delta_theta_avg = (delta_theta_left + delta_theta_right) / 2.0
    
    avance_s = WHEEL_RADIUS * delta_theta_avg
    
    # Actualizamos valores anteriores
    prev_left_enc = current_left_enc
    prev_right_enc = current_right_enc
    
    # --- Filtro de Kalman ---
    d_k_est, P_k = apply_kalman_filter(d_k_est, P_k, z_k, avance_s, Q, R)

    # Lectura de los sensores laterales para la navegación
    raw_left = ps5.getValue()
    raw_right = ps2.getValue()

    # Para las diferencias entre mediciones crudas, filtradas y fusionadas
    if MODO_DATOS == 'crudo':
        # Convertimos la cruda a metros para que la lógica de navegación funcione igual
        distancia_navegacion = convertir_sensor_a_metros(valor_peligro_maximo) 
    elif MODO_DATOS == 'filtrado':
        distancia_navegacion = z_k # z_k ya es la filtrada en metros
    elif MODO_DATOS == 'kalman':
        distancia_navegacion = d_k_est # Estimación del filtro
    else:
        distancia_navegacion = 0.10 # Valor por defecto (10 cm)

    # Llamar a la función de navegación reactiva para decidir las velocidades de los motores
    left_speed, right_speed, modo_evasion, ultima_direccion_giro = compute_motor_speeds(
        front_distance_meters=distancia_navegacion, 
        left_sensor_raw=raw_left, 
        right_sensor_raw=raw_right, 
        max_speed=MAX_SPEED,
        modo_evasion_actual=modo_evasion,
        ultima_direccion_actual=ultima_direccion_giro
    )
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    # --- Almacenamiento de Señales ---
    writer.writerow([
        round(tiempo_actual, 3), 
        round(valor_peligro_maximo, 2), 
        round(sensor_filtrado_crudo, 2), 
        round(z_k, 4), 
        round(avance_s, 5), 
        round(d_k_est, 4)
    ])
