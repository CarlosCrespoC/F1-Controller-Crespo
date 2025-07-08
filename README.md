# Proyecto del primer parcial, controlador para F1TENTH

https://youtu.be/Hi49BySh8bQ

**Repositorio con los archivos del controlador y el F1TENTH GYM para ROS 2 HUMBLE**

Controlador por medio de follow de gap
- Uso y del LIDAR simulado para reconocimiento de obstaculos cercanos.
- Contador de vueltas y cronometro para saber el tiempo que toma en dar cada vuelta
- Procesamiento de datos mediante ajustes finos
- Aplicacion de controlador PID para mejorar la estabilidad a la hora de seguir gaps


Este repositorio contiene nodos de  ROS 2 que implementan un controlador reactivo basado en Follow the Gap, diseñado para vehículos tipo F1TENTH. 
El sistema permite navegar de forma segura por un entorno con obstáculos, seleccionando el mejor camino libre y ajustando su velocidad y dirección en tiempo real.


1. **Detectar obstáculos** directamente al frente.
2. **Identificar huecos (gaps)** suficientemente amplios por donde pueda avanzar.
3. **Calcular el mejor gap** considerando su ancho y cercanía al centro del campo visual.
4. **Aplicar control PID** para ajustar el ángulo de dirección hacia el centro del gap.
5. **Ajustar la velocidad** según la curvatura del giro y la cercanía de obstáculos.


# Explicación detallada del archivo `controller.py`

Este archivo implementa un nodo en ROS 2 que permite el control autónomo de un vehículo tipo F1TENTH mediante un enfoque reactivo. Utiliza información del sensor LiDAR para tomar decisiones en tiempo real sobre la dirección y velocidad, y cuenta las vueltas que el vehículo completa en un circuito cerrado.

---

## Objetivo del nodo

El objetivo principal de este nodo es:
- Detectar obstáculos en el entorno usando un sensor LiDAR.
- Identificar huecos (gaps) suficientemente amplios por donde el vehículo puede pasar.
- Utilizar un controlador PID para girar hacia el centro del gap más seguro.
- Adaptar la velocidad según la curvatura del giro y la proximidad de obstáculos.
- Contar vueltas completadas usando la odometría y medir el tiempo de cada vuelta.

---

## Estructura general

El código está organizado en una clase llamada `ControladorCarrera`, la cual hereda de `rclpy.node.Node`. Dentro de esta clase se definen los siguientes componentes clave:

### 1. Constructor (`__init__`)
- Inicializa el nodo con nombre `controlador_carrera`.
- Crea suscripciones a los tópicos `/scan` (sensor LiDAR) y `/ego_racecar/odom` (odometría).
- Crea un publicador para enviar comandos de velocidad y dirección al tópico `/drive`.
- Configura los parámetros del controlador PID (kp, kd, zona muerta, ángulo máximo).
- Inicia un temporizador que muestra cada segundo el tiempo parcial de la vuelta en curso.

### 2. `procesar_odom(self, msg)`
- Obtiene la posición actual del vehículo a partir del mensaje de odometría.
- Calcula la distancia al origen.
- Si el vehículo entra por primera vez en una zona de 2 metros desde el origen, se cuenta una vuelta.
- Calcula el tiempo de la vuelta y lo registra.
- Usa una bandera `ya_paso_por_origen` para evitar múltiples conteos por ruido o fluctuaciones.

### 3. `mostrar_tiempo_parcial(self)`
- Se ejecuta automáticamente cada segundo.
- Muestra por consola el tiempo que ha pasado desde el inicio de la vuelta actual.
- También imprime la velocidad lineal del vehículo calculada con la odometría.

### 4. `procesar_lidar(self, msg)`
Esta es la función principal de navegación reactiva:
- Limpia y normaliza los datos del LiDAR reemplazando `NaN` por el rango máximo.
- Define una ventana frontal de 270 muestras centradas al frente del vehículo.
- Calcula la distancia mínima en el centro frontal para detectar obstáculos.
- Si hay un obstáculo muy cerca (menos de 0.5 m), el vehículo se detiene.
- Si hay un obstáculo moderadamente cerca (menos de 0.8 m), se aplica evasión girando hacia el lado más despejado.
- Si no hay obstáculos críticos, busca gaps (huecos amplios) en el vector frontal.
- El mejor gap se selecciona considerando su ancho y qué tan centrado está.
- Se calcula el error angular entre el centro del vehículo y el centro del gap.
- Se aplica un controlador PID sobre ese error para obtener el ángulo de dirección.
- La velocidad se ajusta en función del ángulo de giro y la distancia mínima frontal, de forma que se acelera en rectas y se reduce en curvas u obstáculos.

### 5. `enviar_comando(self, velocidad, angulo)`
- Publica un mensaje `AckermannDriveStamped` al tópico `/drive` con la velocidad y el ángulo calculado.
- Se encarga de la comunicación efectiva con el actuador del vehículo.

### 6. `main(args=None)`
- Inicializa el nodo con `rclpy`.
- Ejecuta `rclpy.spin(node)` para mantener el nodo corriendo.
- Al finalizar, destruye el nodo y apaga ROS 2.

---

## Consideraciones adicionales

- El enfoque utilizado es reactivo, por lo que no depende de mapas ni planificación previa.
- El código está diseñado para ser simple y efectivo en pistas cerradas o en entornos con obstáculos dinámicos.
- Puede utilizarse tanto en simuladores como en hardware real (por ejemplo, con un vehículo F1TENTH).

---

# Instrucciones de Uso

Este proyecto está diseñado para funcionar con el entorno de simulación `f1tenth_gym_ros` y un controlador reactivo implementado en el paquete `f1tenth_controller`. A continuación se detallan los pasos para ejecutar la simulación y activar el controlador.

---

## Requisitos previos

Antes de ejecutar, asegúrate de:

- Tener un workspace de ROS 2 correctamente configurado.
- Haber clonado e instalado correctamente los paquetes:
  - `f1tenth_gym_ros`
  - `f1tenth_controller`
- Haber ejecutado la compilación del workspace con `colcon build`.
- Haber cargado el entorno con `source install/setup.bash`.

---

## Comando para iniciar la simulación

Este comando lanza el entorno de simulación utilizando `f1tenth_gym_ros`:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Esto iniciará el simulador F1TENTH y habilitará los tópicos necesarios como `/scan` (LiDAR) y `/ego_racecar/odom` (odometría).

---

## Comando para iniciar el controlador

Una vez que la simulación esté corriendo, en otra terminal ejecuta el controlador reactivo:

```bash
ros2 run f1tenth_controller controller
```

Este comando inicia el nodo que:
- Procesa los datos del sensor LiDAR.
- Encuentra huecos navegables en el entorno.
- Aplica un controlador PID para girar hacia el centro del hueco.
- Controla la velocidad en función de la trayectoria y proximidad de obstáculos.
- Cuenta vueltas y mide el tiempo por vuelta basándose en la odometría.

---

## Consideraciones

- Asegúrate de tener las terminales `sourceadas` correctamente para que los paquetes y nodos se encuentren disponibles.
- El controlador está diseñado para comenzar a tomar decisiones unos segundos después del arranque, para evitar falsos positivos por ruido inicial.

---

## Apagado

Para detener los procesos:

- Usa `Ctrl+C` en ambas terminales (simulador y controlador).
