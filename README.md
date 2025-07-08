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


