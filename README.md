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
* **Robot Empleado:** Robot móvil diferencial e-puck de código abierto.
* **Sensores de Distancia Frontales:** Sensores infrarrojos `ps0` y `ps7` encargados de capturar la proximidad frontal de los obstáculos. Su rango de lectura cruda oscila entre 0 (despejado, ~0.10 m) y 4095 (colisión inminente, ~0.005 m).
* **Sensores de Distancia Laterales:** Sensores `ps2` (derecho) y `ps5` (izquierdo). Se utilizan con un doble propósito: control proporcional de centrado en pasillos durante la marcha recta y toma de decisiones de evasión ante bloqueos frontales.
* **Encoders de Rueda:** Sensores de posición angular `left wheel sensor` y `right wheel sensor`, con los que se mide el desplazamiento acumulado de cada rueda en radianes para calcular de manera precisa el avance lineal del robot a través de odometría.

---

## 3. Frecuencia de Muestreo y Registro de Datos
* **Tiempo de muestreo ($T_s$):** 32 ms (definido dinámicamente mediante el `basicTimeStep` del mundo en Webots).
* **Frecuencia de muestreo ($f_s$):** 31.25 Hz ($f_s = 1 / T_s$).
* **Cantidad de muestras registradas:** Aproximadamente 6,250 filas de datos recolectadas en un experimento continuo de 200 segundos de simulación activa, almacenadas en el archivo 'datos_sensores.csv'.

---

## 4. Estimación del Avance mediante Encoders
El desplazamiento lineal de cada rueda se calcula en cada paso de simulación a partir de la variación de la posición angular medida por los encoders ($\Delta\theta$), utilizando la relación cinemática de la rodadura:

$$s = r \cdot \Delta\theta$$

Donde $r = 0.0205\text{ m}$ representa el radio de las ruedas del e-puck. 

El avance total del robot ($\Delta d_k$) en un intervalo de tiempo se estima promediando el avance lineal de ambas ruedas:

$$\Delta d_k = \frac{s_{\text{izq}} + s_{\text{der}}}{2}$$

Este desplazamiento incremental es el núcleo de la etapa de predicción geométrica del Filtro de Kalman, asumiendo una trayectoria rectilínea hacia el obstáculo cuando el robot navega de frente.

---

## 5. Filtrado Simple Aplicado
Antes de ingresar al Filtro de Kalman, la lectura de hardware pasa por un procesamiento previo:
* **Fusión por Máximo:** Se toma el valor máximo entre `ps0` y `ps7` $(Valor = \max(ps0, ps7))$. Esto asegura una postura conservadora de seguridad: si un obstáculo ingresa en diagonal por un flanco, el robot reacciona inmediatamente aunque el otro sensor apunte al vacío.
* **Filtro de Promedio Móvil:** Se aplica una ventana deslizante de 3 muestras sobre el valor máximo obtenido para mitigar los picos de ruido blanco de alta frecuencia intrínsecos de los infrarrojos simulados en Webots.
* **Conversión a metros:** La señal filtrada en formato crudo se linealiza y se convierte a unidades métricas mediante una función calibrada para el e-puck, mapeando el rango dinámico del robot desde $0.005\text{ m}$ (contacto) hasta un horizonte máximo de despeje de $0.10\text{ m}$ ($10\text{ cm}$).

---

## 6. Implementación del Filtro de Kalman
Se diseñó e implementó un Filtro de Kalman Escalar dinámico para fusionar la información predictiva de la odometría (encoders) con las observaciones físicas del entorno (sensores infrarrojos de distancia). El filtro opera secuencialmente en tiempo real bajo dos etapas principales en cada paso de simulación:

### A. Etapa de Predicción (Actualización del Estado por Modelo)
En esta fase, el filtro proyecta cuál debería ser la nueva distancia hacia el obstáculo basándose exclusivamente en el movimiento del robot. **Es crítico destacar que la predicción se congela si el robot está girando sobre su propio eje**, ya que la rotación pura no altera la distancia lineal hacia la pared frontal. Si el robot se desplaza en línea recta, se resta el avance medido por los encoders.

A partir del estado previo $\hat{d}_ {k-1}$ y su covarianza del error $P_{k-1}$, se computa el estado predicho $\hat{d}_ {k}^{-}$ y la covarianza predicha $P_{k}^{-}$, incorporando la incertidumbre del modelo cinemático ($Q$):

$$\hat{d}_{k}^{-} = \hat{d}_{k-1} - \Delta d_{k}$$

$$P_{k}^{-} = P_{k-1} + Q$$

### B. Etapa de Corrección (Actualización del Estado por Medición)
Una vez que se obtiene la medición real del sensor ($z_k$) convertida a metros, el filtro calcula la Ganancia de Kalman ($K_k$). Esta matriz escalar actúa como un ponderador dinámico que decide a qué señal hacerle más caso (si al modelo geométrico o a la lectura del sensor). Finalmente, se corrige la estimación de la distancia ($\hat{d}_{k}$) y se actualiza la covarianza del error ($P_k$) para el siguiente ciclo:

$$K_k = \frac{P_{k}^{-}}{P_{k}^{-} + R}$$

$$\hat{d}_{k} = \hat{d}_{k}^{-} + K_k \cdot (z_k - \hat{d}_{k}^{-})$$

$$P_k = (1 - K_k) \cdot P_{k}^{-}$$

### Parámetros de Ajuste Utilizados:
* **Incertidumbre del Proceso ($Q$):** `0.01` (Representa la confianza en la precisión geométrica de los encoders).
* **Varianza del Ruido del Sensor ($R$):** `0.02` (Representa la magnitud de las fluctuaciones físicas y el ruido de alta frecuencia del sensor analógico).

---
## 7. Lógica de Navegación Reactiva
El control combina una estrategia proporcional para el mantenimiento de pasillos y una secuencia basada en tiempo/ciclos para las maniobras de evasión de 90°:

1. **Navegación Recta con Control Lateral Proporcional:** Mientras el frente está despejado, el robot avanza a una velocidad crucero ($0.5 \cdot MAX\_SPEED$). Si los sensores laterales (`ps2` o `ps5`) superan el ruido base de $80.0$, se calcula un error de centrado ($Error = ps2 - ps5$). Un controlador **KP = 0.003** ajusta de forma diferencial los motores, permitiendo al robot alejarse de las paredes laterales y navegar en perfecta estabilidad por el centro de pasillos estrechos sin rozar.
2. **Activación de Evasión:** Si la distancia métrica final estimada por Kalman cae por debajo de los $0.05\text{ m}$ ($5\text{ cm}$), el robot interrumpe la marcha y entra en modo evasión.
3. **Fase de Despegue:** El e-puck retrocede de forma rectilínea durante exactamente 10 ciclos para despegarse del obstáculo y adquirir espacio de maniobra.
4. **Decisión de Dirección y Giro de 90°:** Al finalizar el retroceso, se comparan los sensores laterales. Si $ps5 > ps2$, el obstáculo lateral está a la izquierda, por lo que se ejecuta un giro horario sobre su propio eje (hacia la derecha). En caso contrario, gira a la izquierda. La rotación se mantiene fija durante 45 ciclos de reloj para garantizar un ángulo limpio de 90° antes de restablecer el modo de avance recto.

---

## 8. Análisis de Señales y Gráficos

A continuación, se presentan las gráficas comparativas de las señales durante las pruebas:

### Comparativa de Señales Frontales en Tiempo Real
![Comparativa de Señales](multimedia/grafico_entornosimple.png)

### Análisis Detallado del Gráfico (Muestreo de 200 Segundos):
Al evaluar la gráfica obtenida a partir del archivo de logs, se evidencian de forma clara las ventajas de la fusión sensorial frente al procesamiento convencional:

* **Supresión Eficiente del Ruido Blanco:** En los tramos horizontales superiores (por ejemplo, entre los segundos 10 y 65 o entre 165 y 180), la línea naranja (`Sensor_Metros`) muestra un comportamiento varianza de alta frecuencia debido a las interferencias inherentes del sensor. En contraste, la línea azul (`Kalman_Distancia`) cruza de manera sólida y completamente suavizada por el centro de la nube de datos, demostrando que el filtro discrimina de forma óptima el ruido eléctrico y ambiental, previniendo micro-oscilaciones en los motores.
* **Respuesta Dinámica Ante Obstáculos:** Periódicamente (en los segundos 65, 80, 95, 110, 128, etc.), la distancia cae abruptamente en caída libre. Estos eventos representan el momento exacto en que el robot encuentra una pared al frente. La línea azul acompaña la caída de la línea naranja sin retrasos significativos (*time-lags*), lo que comprueba que la Ganancia de Kalman ($K_k$) se eleva correctamente priorizando la medición real cuando el entorno cambia de forma imprevista, asegurando una evasión oportuna.
* **Estabilización en Espacio Despejado:** Cuando el robot completa su giro de 90° y el frente queda libre, ambas señales regresan al límite superior calibrado de $0.05\text{ m}$, manteniendo al robot en velocidad de crucero constante hasta el próximo muro.

---

## 9. Resultados en los Escenarios de Prueba
Para evaluar el desempeño de la navegación reactiva fusionada, se diseñaron y ejecutaron simulaciones en dos entornos con niveles de dificultad incremental dentro de Webots:

### A. Entorno 1: Ambiente Simple (Pocos obstáculos)
![Entorno Simple](multimedia/entornosimple.png)

* **Descripción:** Espacio delimitado con obstáculos cilíndricos aislados y amplias zonas despejadas para el tránsito.
* **Comportamiento del Robot y Estabilidad:** El e-puck mostró un comportamiento impecable. Avanzó en trayectorias rectas perfectas hacia los cilindros. Al cruzar el umbral de los 5 cm, realizó la maniobra de retroceso y giro de escape con precisión, retomando la navegación rectilínea inmediatamente después sin registrar titubeos ni giros falsos en zonas abiertas.

### B. Entorno 2: Ambiente Complejo (Pasillos estrechos y giros cerrados)
![Entorno Complejo](escenario_complejo.jpg)

* **Descripción:**
* **Comportamiento del Robot y Capacidad de Evasión:** 

---

## 10. Análisis Final y Conclusiones
* **Conclusión sobre los tipos de señal:** El uso de lecturas crudas en robótica reactiva es inviable a nivel práctico dado que el ruido provoca activaciones falsas de evasión o frenadas bruscas. Si bien un promedio móvil suaviza la señal, introduce un retardo temporal perjudicial a altas velocidades. La fusión sensorial mediante el Filtro de Kalman resolvió el dilema al balancear la predictibilidad matemática de la odometría con la realidad física del sensor infrarrojo, entregando una señal limpia y de respuesta instantánea.
* **Lecciones Aprendidas:** Se constató que el Filtro de Kalman debe ajustarse en estrecha relación con el estado del vehículo. Restar el avance de las ruedas durante un giro estático induce errores graves de estimación en el filtro. Al aislar cinemáticamente las etapas de predicción según el estado de movimiento (recto o giro), el filtro adquiere robustez conceptual y matemática, permitiendo un comportamiento autónomo fiable en entornos de complejidad incremental.

---

## 11. Instrucciones para Ejecutar la Simulación
Para replicar las pruebas de este controlador, siga los siguientes pasos:

1. Descargue o clone este repositorio en su máquina local:
   ```bash
   git clone [https://github.com/TheFanking/Laboratorio-2.git]
