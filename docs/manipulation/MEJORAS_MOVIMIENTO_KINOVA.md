# Documentación de Mejoras de Suavidad y Fluidez del Kinova
En este documento explicamos las estrategias incorporadas para resolver vibraciones intermitentes (stutterings) y aceleraciones súbitas basándonos en la configuración encontrada en ControlKinova.

## 1. Modificación de Perfiles de Aceleración y Velocidad (MTC/Planificador)
En cualquier momento que construyas un movimiento con `MoveGroup` o en sus planificadores internos por MTC (`MoveIt Task Constructor`), debes forzar un escalado de la velocidad y aceleración al arrancar una trayectoria, limitándolo alrededor del 5% al 15%.
Esto garantiza arranques curvos en lugar de saltos instantáneos al 100% de la energía máxima disponible.

**Ejemplo de cómo adaptarlo en Python / Acción de Cliente MTC**:
```python
goal_msg.request.max_velocity_scaling_factor = 0.05      # 5% de la Vel máxima de URDF
goal_msg.request.max_acceleration_scaling_factor = 0.05  # 5% de Ac de empuje
```
Esta reducción es imperativa en el entorno de Pick And Place logístico.

## 2. Driver de Hardware y Rutas UDP a Baja Latencia (Realizadas vía `scripts/apply_kinova_smooth_movement.py`)
Por defecto, la instalación desde las fuentes de `ros2_kortex` que corre a nivel de Sistema Operativo implementaba tolerancias de desconexiones al robot sub-óptimas.
Se construyó un parche (en Python) que modifica explícitamente `hardware_interface.cpp` dentro del driver de hardware (`ros2_kortex/kortex_driver/src/hardware_interface.cpp`), insertando un Timeout para el router de mensajería (RouterClientSendOptions) en `200 ms`. Al purgar estas demoras en el código en C++ original, el brazo pierde ese "Lag" entre que MoveIt envía la trayectoria por red ROS y los motores de Kinova la obedecen.

## 3. Topología de Red Interna de los Motores: `use_internal_bus_gripper_comm`
Un factor desestabilizante de control de movimiento al enviar "Poses o Gripper Comm" se daba porque la simulación o el URDF asumían dos vías de comunicación diferentes (brazo y pinza separados).
El script inyecta la instrucción `use_internal_bus_gripper_comm="true"` en los URDF natales de `ros2_kortex/kortex_description`. 
Esto previene que la Pinza intente responder de forma desincronizada a la red, resultando en oscilaciones espaciales durante las tareas de manipulación al mover e ir abriendo el efector final simultáneamente.

Asegúrate de haber ejecutado el script en tu terminal con:
```bash
python3 /home/roncanciovl/ros2_ws/src/burger_delivery/scripts/apply_kinova_smooth_movement.py
```
Y haber reconstruido el entorno local.

Y recuerda añadir estas optimizaciones y restricciones en futuras implementaciones de Python del `burger_delivery`!
