# Laboratorio 2: Navegación Reactiva con Filtrado y Fusión de Sensores en Webots
## Asignatura: Robótica y Sistemas Autónomos (ICI 4150)

### Integrantes:
* Alfredo Escobar
* José Mena
* Branco González
* Michelle Hernández

---

## 1. Objetivo del Trabajo
El objetivo de este laboratorio es implementar un sistema de navegación reactiva para un robot móvil diferencial (e-puck) en el entorno de simulación Webots. Se busca integrar sensores de distancia y encoders de rueda mediante técnicas de filtrado móvil y fusión sensorial con un Filtro de Kalman Escalar. Con esto, se estima la distancia frontal hacia los obstáculos más cercanos de forma robusta frente al ruido para optimizar la toma de decisiones del robot.

---

## 2. Descripción del Robot y Sensores Utilizados
* **Robot Empleado:** Robot móvil diferencial e-puck.
* **Sensores de Distancia Frontales:** Sensores infrarrojos ps0 y ps7 encargados de capturar la proximidad frontal de los obstáculos. Su rango de lectura cruda oscila entre 0 (lejos) y 4095 (cerca).
* **Sensores de Distancia Laterales:** Sensores ps2 (derecho) y ps5 (izquierdo) para determinar la dirección de giro en maniobras de evasión.
* **Encoders de Rueda:** Sensores de posición angular left wheel sensor y right wheel sensor, con los que se mide el desplazamiento de cada rueda en radianes para calcular el avance lineal del robot.

---

## 3. Frecuencia de Muestreo y Registro de Datos
* **Tiempo de muestreo (Ts):** 32 ms (definido dinámicamente mediante el basicTimeStep del mundo en Webots).
* **Frecuencia de muestreo (fs):** 31.25 Hz (fs = 1 / Ts).
* **Cantidad de muestras registradas:** [Indicar el número de muestras o filas finales guardadas en tu archivo CSV por experimento].

---

## 4. Estimación del Avance mediante Encoders
El desplazamiento lineal de cada rueda se calcula a partir de la variación de la posición angular medida por los encoders (Delta Theta), utilizando la relación cinemática:

s = r * Delta_Theta

Donde r = 0.0205 m representa el radio de las ruedas del e-puck. 

El avance total del robot (Delta d_k) en un intervalo de tiempo se estima promediando el avance lineal de ambas ruedas:
Delta d_k = (s_izq + s_der) / 2

Este valor es fundamental, ya que alimenta la etapa de predicción geométrica del Filtro de Kalman.

---

## 5. Filtrado Simple Aplicado
Antes de procesar la señal en el Filtro de Kalman, se aplica un Filtro de Promedio Móvil con una ventana de 3 muestras sobre el valor máximo registrado por los sensores frontales (ps0 y ps7). 
* **Justificación:** Los sensores infrarrojos sufren de picos de ruido de alta frecuencia en Webots. Tomar el valor máximo previene colisiones si un sensor detecta el obstáculo antes que el otro, y el promedio móvil suaviza la transición de la señal.
* **Conversión a metros:** La señal filtrada en formato crudo se convierte a una escala lineal aproximada en metros mediante la función convertir_sensor_a_metros(), asumiendo una distancia máxima de detección de 0.05 m (5 cm).

---

## 6. Implementación del Filtro de Kalman
Se diseñó un Filtro de Kalman Escalar para fusionar la odometría (encoders) con las lecturas directas del entorno (sensores de distancia). El filtro opera secuencialmente bajo dos etapas principales:

### A. Etapa de Predicción (Movimiento)
Se proyecta el estado de la distancia frontal estimada restando el avance lineal calculado por los encoders, actualizando además la covarianza del error con el ruido del proceso (Q):
Distancia_Predicha = Distancia_Anterior - Avance_Encoders
Covarianza_Predicha = Covarianza_Anterior + Q

### B. Etapa de Corrección (Medición)
Se calcula la Ganancia de Kalman (K) ponderando la incertidumbre de la predicción frente a la varianza del ruido del sensor (R). Posteriormente, se actualiza el estado estimado e introduce la nueva covarianza corregida:
K = Covarianza_Predicha / (Covarianza_Predicha + R)
Distancia_Estimada = Covarianza_Predicha + K * (Medicion_Sensor - Distancia_Predicha)
Covarianza_Actualizada = (1 - K) * Covarianza_Predicha

* **Parámetros de Ajuste Utilizados:** Q = 0.06 (Ruido del proceso) y R = 0.02 (Varianza del ruido del sensor).

---

## 7. Lógica de Navegación Reactiva
La toma de decisiones del robot se basa en una máquina de estados con umbrales de histéresis para evitar oscilaciones o comportamientos erráticos:
* **Estado Avanzar:** El robot se desplaza en línea recta a velocidad crucero (0.5 * MAX_SPEED) mientras la distancia frontal estimada sea superior al umbral de seguridad (0.045 m).
* **Estado Evasión:** Se activa si la distancia estimada cae por debajo de 0.045 m. Para salir de este estado, el frente debe despejarse por completo superando los 0.048 m.
* **Decisión de Giro:** Si hay peligro al frente, se evalúan los sensores laterales:
  * Si ps5 (izquierdo) > ps2 (derecho), el obstáculo está más cerca por la izquierda; el robot rota sobre su propio eje hacia la derecha.
  * Si ps2 > ps5, rota hacia la izquierda.
  * Se implementó una banda muerta (LATERAL_DEADBAND_RAW = 35.0) para preservar el último sentido de giro si el obstáculo está perfectamente centrado, eliminando fluctuaciones rápidas.

---

## 8. Análisis de Señales y Gráficos

A continuación, se presentan las gráficas comparativas de las señales durante las pruebas:

### [Insertar Gráfico 1: Comparativa de Señales Frontales]
![Comparativa de Señales](grafico_sensores.png)

### Análisis del Gráfico:
Al observar el comportamiento de las señales temporales capturadas durante el experimento, se pueden identificar con claridad las tres fases del trayecto del robot y cómo responde cada algoritmo:
* Fase 1: Transición Inicial (Segundos 0 a 2.5): Al arrancar el controlador, se aprecia que la medición cruda del sensor (Sensor_Metros en naranja) presenta una lectura artificialmente baja (en torno a $0.005\text{ m}$). Esto coincide plenamente con la advertencia del código sobre los valores iniciales extraños de los sensores. Lo destacable aquí es ver cómo el Filtro de Kalman (Kalman_Distancia en azul) ignora esa lectura errónea gracias a que la covarianza inicial y la predicción por odometría la estabilizan, evitando que el robot realice un giro falso al iniciar la simulación.
* Fase 2: Aproximación en Línea Recta (Segundos 2.5 a 12.5): Durante este tramo, el robot se desplaza de forma constante hacia un obstáculo frontal. La curva azul de Kalman decrece de forma perfectamente lineal y suave. Esto ocurre porque el filtro fusiona la información del avance constante calculado por los encoders de las ruedas con las lecturas ruidosas del sensor. La línea naranja (cruda) muestra picos y dientes de sierra debido al ruido inherente de los infrarrojos de Webots, pero la estimación fusionada (azul) filtra por completo esas perturbaciones de alta frecuencia.
* Fase 3: Maniobra de Evasión (Segundos 12.5 a 15):En el segundo 12.5, la distancia estimada cruza con precisión el umbral de evasión establecido en $0.045\text{ m}$. Inmediatamente, la lógica reactiva conmuta los motores para iniciar el giro de escape. En este punto, la distancia al obstáculo cambia drásticamente; la señal cruda naranja salta de forma abrupta hacia los $0.05\text{ m}$ (máximo rango) cuando el frente se despeja, mientras que el Filtro de Kalman acompaña esa transición de manera controlada y amortiguada, garantizando que el robot no vuelva a oscilar ni a tomar decisiones erráticas debido a cambios súbitos en las lecturas.

---

## 9. Resultados en los Escenarios de Prueba
Para evaluar el desempeño de la navegación reactiva fusionada, se diseñaron y ejecutaron simulaciones en dos entornos con niveles de dificultad incremental dentro de Webots:

### A. Entorno 1: Ambiente Simple (Pocos obstáculos)
![Entorno Simple](escenario_simple.jpg)

* **Descripción:** Este escenario consiste en un espacio abierto delimitado donde se posicionaron obstáculos cilíndricos aislados y distanciados entre sí. El robot dispone de zonas despejadas de tránsito antes de encontrarse con una colisión frontal.
* **Comportamiento del Robot y Estabilidad:** El e-puck mostró un desplazamiento rectilíneo altamente estable. Al aproximarse a un cilindro de forma perpendicular, la distancia estimada por el Filtro de Kalman disminuyó de manera suave y continua. Al cruzar el umbral de seguridad, el robot realizó rotaciones limpias hacia el flanco con mayor espacio libre (determinado por los sensores ps2 y ps5) y reanudó la marcha sin registrar oscilaciones, frenadas intermitentes ni giros falsos.

### B. Entorno 2: Ambiente Complejo (Pasillos estrechos y giros cerrados)
![Entorno Complejo](escenario_complejo.jpg)

* **Descripción:** Este escenario simula una pista cerrada o laberinto caracterizado por paredes continuas, esquinas en ángulo recto, pasillos estrechos y una mayor densidad de obstáculos cilíndricos interconectados que limitan el espacio de maniobra.
* **Comportamiento del Robot y Capacidad de Evasión:** 

---

## 10. Análisis Final y Conclusiones
* **Conclusión sobre los tipos de señal:** La señal cruda de los sensores infrarrojos por sí sola provoca detenciones bruscas o giros innecesarios debido a falsos positivos generados por el ruido. El promedio móvil reduce el ruido pero introduce un leve retardo. Por su parte, la fusión sensorial con el Filtro de Kalman provee la estimación más balanceada al predecir el cambio de distancia mediante el desplazamiento real medido por la odometría.
* **Lecciones Aprendidas:** Ajustar correctamente las matrices de covarianza y ruido (Q y R) es fundamental; subestimar el ruido del sensor provoca que el filtro herede las fluctuaciones no deseadas del entorno, mientras que sobreestimarlo vuelve al robot insensible ante obstáculos dinámicos.

---

## 11. Instrucciones para Ejecutar la Simulación
Para replicar las pruebas de este controlador, siga los siguientes pasos:

1. Descargue o clone este repositorio en su máquina local:
   ```bash
   git clone [https://github.com/TheFanking/Laboratorio-2.git]
