# Arquitectura de Control Brazo Robótico (MoveIt 2 & ros2_control)

Para entender cómo se mueve un robot articulado como el **Kinova Gen3 7DOF** en un entorno ROS 2 complejo (como el de este laboratorio de *Burger Delivery*), es esencial separar el control en **tres grandes cerebros**.

A diferencia de un carro a RC donde envías voltajes a los motores directamente, los brazos robóticos operan mediante un ciclo estricto de **Planificación -> Ejecución -> Hardware**. Estos tres cerebros son:

---

## 1. El Cerebro Inteligente: **MoveIt 2**
*Es el "Google Maps" o "Waze" del brazo robótico.*

### El Problema
Si quieres mover la pinza del brazo hacia una caja de hamburguesas ubicada en coordenadas XYZ exactas —por ejemplo `(0.4, 0.2, 0.3)`—, ¿cuántos grados debe rotar el hombro? ¿y el codo? ¿y la muñeca? 
Esa matemática no se puede adivinar ni programar a mano porque hay **infinitas combinaciones** para los 7 motores.

### La Solución de MoveIt 2
MoveIt recibe la "meta tridimensional" o `Goal Pose` y hace lo siguiente por debajo de la mesa:
1. **Cinemática Inversa (IK):** Calcula matemáticamente los ángulos exactos que necesita la articulación 1 a la 7 para que la pinza llegue a esa coordenada espacial exacta.
2. **Evasión de Colisiones (Collision Avoidance):** Lee el URDF e infla obstáculos virtuales (como mesas virtuales o al propio robot). Si calcular la ruta implica chocar el robot consigo mismo (ej. enterrarse la pinza en la propia base), descarta esa trayectoria y calcula una nueva ruta esquivando los obstáculos.
3. **Generación de Curvas Suaves (Splines):** No le dice al motor "Salta de 0 a 90 grados al instante", porque destruirías el motor real. Le dice al hardware: *"Sube 1 grado el primer milisegundo, luego acelera a 5 grados, luego frena..."*, creando aceleraciones y desaceleraciones perfectas en forma de curva *trapezoidal*.

**Resumen:** Genera una hoja de ruta perfecta.

---

## 2. El Intermediario Intérprete: **ros2_control**
*Es el músculo y los nervios del sistema.*

### El Problema
MoveIt ha terminado su trabajo; ahora tiene un archivo larguísimo que dice cómo deben moverse los motores en cada segundo. Pero a MoveIt no le importa si tu brazo es marca Kinova, Universal Robots, KUKA, o un brazo hechizo de madera y servomotores chinos; MoveIt no sabe cómo hablar con piezas electrónicas ni voltajes.

### La Solución de ros2_control
Es un traductor estandarizado. Consiste en varios **Controller Managers** y funciona en dos vías:
*   **Recibe las órdenes de alta calidad de MoveIt** (la trayectoria de vértices pre-calculados, como `/joint_trajectory_controller`).
*   **Traducción:** Envía al Driver oficial del robot comandos simples en el idioma que él pueda leer.
*   **Aparato Excretor:** Permite a herramientas visuales como **rqt_joint_trajectory** (el panel de los deslizadores) ignorar a MoveIt por completo y secuestrar directamente los motores en modo manual.

**Resumen:** Enruta las trayectorias de MoveIt a la tarjeta madre electrónica correspondiente.

---

## 3. El Cerebro de Fondo: Driver & API (kortex_driver)
*Es el kernel exclusivo de la marca.*

### Qué ocurre en la capa final electrónica
La empresa Kinova programó en C++ una API propietaria llamada **`kortex_api`**.
Cuando **`ros2_control`** envía la rotación de "5 radianes en el codo", el driver empaqueta esa información en la capa TCP/IP como un paquete de red UDP/Ethernet con el protocolo cerrado secreto de Kinova y se lo envía por el cable gris oscuro de red físicamente al robot con IP `192.168.1.10`.

Además, si un humano empuja el robot con las manos, y un encoder del hardware lee que la posición se desvió, **`kortex_driver`** le grita por el cable a **`ros2_control`** que se ha movido, este actualiza los valores y finalmente en RViz ves en vivo cómo la malla 3D de la pantalla se mueve mientras tú empujas el robot real.

---

### Conclusión para tu Tarea Diaria en el Proyecto:

Sabiendo esto, ahora cada vez que un componente falla sabes a quién culpar o dónde arreglarlo:

1.  *¿El robot real da un salto brusco e hiperagresivo?* = Error de sintonización en el generador de de movimientos de **MoveIt 2**.
2.  *¿RViz muestra al fantasma naranja atravesando la pared de la mesa que creaste?* = Error de configuración de colisiones en la escena de **MoveIt**.
3.  *¿El robot intenta rotar articulaciones imposibles cuando usas el panel deslizador manual?* = MoveIt no está cuidándote las espaldas porque te saltaste ese paso para interactuar directamente con **ros2_control**.
4.  *¿Te da un error "Overrun (1000hz)... missed loops"?* = El cable o la comunicación hacia tu **`kortex_driver`** está sufriendo latencias, provocando que los nervios reaccionen tarde a las órdenes. *(Nota: Por ello aplicamos una modificación al `hardware_interface.cpp` reduciendo su timeout a 200 ms evitando que pierda sincronía).*

### ⚙️ Actualizaciones Recientes del Sistema (Visión y Movimiento Fluido)
En adición a MoveIt 2 puro y duro, agregamos dos características invaluables a esta capa de arquitectura:

1. **Restricción de Curvas (Scaling Factors)**: Por defecto, los perfiles arrancan subiendo bruscamente una alta inercia (jerk) sacudiendo el sistema. Hemos establecido políticas de limitar drásticamente `max_velocity_scaling_factor` y `max_acceleration_scaling_factor` usualmente al `0.05` (5%) cada vez que invoquemos un cliente de Acción. Este salto curvo es la clave del transporte suave en Robótica Colaborativa (Cobots).
2. **Ghost Display (Planeamiento Visual Interactivo)**: Enrutamos una copia del RobotModel suscribiéndose internamente vía plugin a la fase de planeación (`MotionPlanning`). Dándole un nivel de opacidad (transparencia Alpha al 50%), ahora puedes visualizar y arrastrar al robot "Fantasma" físicamente dentro de RViz. Él encarna la pose espacial planificada (`Show Goal State`) dándonos certeza 100% algorítmica de la posición IK antes y durante la emisión de la orden física de `kortex_api`.
