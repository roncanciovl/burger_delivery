# 🎓 Guía Paso a Paso: Dominando la Interfaz de Línea de Comandos (CLI) de ROS 2

¡Bienvenido! En esta guía (Taller) daremos nuestros primeros pasos prácticos en el ecosistema de ROS 2. Basándonos en los tutoriales iniciales oficiales de ROS 2 Jazzy, aprenderás a usar las herramientas de terminal (CLI) que te permitirán inspeccionar, depurar y controlar cualquier robot, sea simulado o físico.

## 🎯 Resultado de Aprendizaje Evaluable (RAE)
**RAE 1 (Primer Corte):** *Comprender la arquitectura distribuida, redes, comunicación técnica y experimentación en el ecosistema ROS 2.*
Al finalizar esta guía, deberás completar el componente de evaluación final que valida este resultado.

## 🧠 Contexto: ¿Por qué la terminal es tu mejor amiga?
En ROS 2, no todo tiene una interfaz gráfica bonita con botones. La mayoría de los robots profesionales (como nuestro manipulador Kinova Gen3 o sistemas de visión avanzados) corren internamente en servidores Linux sin pantalla. Para interactuar con ellos, usamos la Interfaz de Línea de Comandos (CLI).

Los comandos que aprenderás aquí usando un simple simulador 2D (`turtlesim`) son **exactamente los mismos** que utilizarás más adelante para diagnosticar el funcionamiento del robot `burger_delivery`.

---

Cada sección de este taller combina:
1. **El Concepto:** ¿Qué es este elemento en la arquitectura ROS 2?
2. **El Ejercicio:** Comandos reales para ejecutar.
3. **Mini-Reto:** Una pequeña validación para asegurar el conocimiento.

¡Vamos allá!

## 0. Preparando el Entorno

### 🧠 El Concepto
Antes de usar cualquier comando de la familia `ros2`, el sistema operativo debe saber dónde están instalados los programas. Esto se llama "hacer source" del entorno.

### 🛠️ Ejercicio
Abre una terminal nueva y ejecuta:
```bash
source /opt/ros/jazzy/setup.bash
```
*(Nota: Si configuraste correctamente tu `~/.bashrc` en el taller de redes, este paso se hará automáticamente al abrir una terminal, pero siempre es bueno saber de dónde viene).*

## 1. Turtlesim y rqt: Tu primer sistema robótico

### 🧠 El Concepto
`turtlesim` es un simulador ligero diseñado específicamente para enseñar los conceptos básicos de ROS 2 sin el riesgo de dañar un robot físico. Por otro lado, `rqt` es una herramienta gráfica que nos permite "ver por debajo del capó" del sistema ROS 2.

### 🛠️ Ejercicio: Lanzar los nodos base
Necesitaremos **tres** terminales distintas para este experimento.

1. **Terminal 1:** Lanza el robot (el simulador).
   ```bash
   ros2 run turtlesim turtlesim_node
   ```
   *Verás aparecer una ventana azul con una tortuga en el centro.*

2. **Terminal 2:** Lanza el control remoto.
   ```bash
   ros2 run turtlesim turtle_teleop_key
   ```
   *Usa las flechas del teclado en esta terminal para mover la tortuga.*

3. **Terminal 3:** Lanza la interfaz de diagnóstico gráfico.
   ```bash
   rqt
   ```

### ✅ Criterio de Éxito
- Eres capaz de mover la tortuga por la pantalla manteniendo la ventana de la Terminal 2 activa.
- Observas las estelas blancas que deja el movimiento.

## 2. Entendiendo los Nodos (Nodes)

### 🧠 El Concepto
Un **nodo** es un pequeño programa (ejecutable) en ROS 2 que realiza una tarea específica (ej. leer un láser, controlar una rueda, o en este caso, dibujar una tortuga en pantalla). El sistema robótico completo es una red (grafo) distribuida de muchos nodos colaborando entre sí.

### 🛠️ Ejercicio: Inspección
En una nueva terminal, vamos a listar los nodos que están vivos:
```bash
ros2 node list
```
*(Deberías ver listados a `/turtlesim` y `/teleop_turtle`)*.

Para ver los detalles íntimos de un nodo específico (qué produce y qué consume):
```bash
ros2 node info /turtlesim
```
Observa las categorías: publicadores (Publishers), suscriptores (Subscribers) y servicios (Services) que este nodo ofrece.

### 🛠️ Entregable 1 (Mini-Reto): La muerte de un nodo
1. Cierra la terminal donde está corriendo el nodo `teleop_turtle` presionando `Ctrl+C`.
2. Ejecuta `ros2 node list` de nuevo. Documenta qué nodo ha desaparecido de la red (captura o texto).
3. Vuelve a lanzar el `teleop_turtle` para continuar.

## 3. Entendiendo los Tópicos (Topics)

### 🧠 El Concepto
Los nodos se pasan información (datos continuos) entre sí a través de **tópicos**. Piensa en los tópicos como canales de televisión. Un nodo puede **publicar** (transmitir) en un canal, y cualquier otro nodo puede **suscribirse** (sintonizar) a ese mismo canal. Es una comunicación asíncrona.

### 🛠️ Ejercicio: Inspección de Tópicos
1. En la ventana de rqt: Ve al menú `Plugins > Introspection > Node Graph`. Verás una representación visual de cómo el nodo `teleop` envía una flecha (el tópico) al nodo `turtlesim`.
2. Lista los tópicos activos desde la terminal:
   ```bash
   ros2 topic list
   ```
3. Ahora vamos a "espiar" la información de velocidad que viaja por el canal:
   ```bash
   ros2 topic echo /turtle1/cmd_vel
   ```
   *(Presiona las flechas en tu terminal de teleop y observa cómo los números se imprimen en tiempo real).*

### 🛠️ Entregable 2 (Mini-Reto): ¡El Desafío de la Letra Cursiva!

> [!NOTE]
> **Fundamento de Robótica: El concepto de "Twist"**
> En la cinemática de cuerpos rígidos, un *Twist* es la representación matemática que expresa la velocidad de un objeto en el espacio 3D. ROS 2 estandariza esto a través del mensaje `geometry_msgs/msg/Twist`, dividiéndolo siempre en dos vectores:
> - **`linear (x, y, z)`**: Velocidad de traslación en metros por segundo (m/s). (Ej. Moverse hacia adelante es `linear.x`).
> - **`angular (x, y, z)`**: Velocidad de rotación en radianes por segundo (rad/s). (Ej. Girar hacia la izquierda/derecha es rotar sobre el eje Z, `angular.z`).
> Todos los robots móviles en ROS 2 usan esta misma estructura para moverse.

Vamos a inyectar comandos directamente desde la terminal, sin usar el teclado. Prueba enviando este mensaje:
```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
*(Al combinar traslación en X y rotación en Z, verás que la tortuga traza un pequeño arco).*

**¡Tu Reto!** Utilizando este comando secuencialmente y modificando los valores de `linear.x` (avance) y `angular.z` (rotación), intenta **dibujar la inicial de tu nombre en letra cursiva** en la pantalla. 
*Pista:* Un `angular.z` positivo gira a la izquierda y uno negativo a la derecha. Si le quitas el parámetro `--once` al comando, el mensaje se publicará de manera continua (muy útil para hacer curvas redondas largas) hasta que lo detengas presionando `Ctrl+C`.

## 4. Entendiendo los Servicios (Services)

### 🧠 El Concepto
Mientras que los tópicos son para flujos continuos de datos, los **Servicios** son para acciones síncronas de "Petición y Respuesta" (Call & Response). Un cliente hace una petición, el servidor detiene lo que hace, procesa la solicitud y devuelve un resultado.

### 🛠️ Ejercicio: Usando Servicios
1. Lista todos los servicios expuestos en la red:
   ```bash
   ros2 service list
   ```
2. Vamos a usar el servicio `/turtle1/teleport_absolute` para enviar ("teletransportar") a la tortuga instantáneamente a una coordenada específica. Esto ilustra perfectamente el propósito de un servicio: enviar una orden directa (discreta) de una sola vez y esperar a que se cumpla de inmediato, a diferencia de los tópicos que son flujos continuos.
   ```bash
   ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 8.0, y: 8.0, theta: 1.57}"
   ```
   *Verás que la tortuga salta de inmediato a la parte superior derecha de la pantalla.*

### 🛠️ Entregable 3 (Mini-Reto): Borrando el rastro
Utiliza el comando de lista de servicios para encontrar el servicio que borra las líneas trazadas en la pantalla. (Pista: el nombre termina en `clear`).
Llámalo ejecutando: `ros2 service call /clear std_srvs/srv/Empty`
Toma una captura evidenciando el comando exitoso y la pantalla limpia.

## 5. Parámetros y Acciones (Vista Rápida)

### 🧠 Conceptos Breves
- **Parámetros (Params):** Son las configuraciones estáticas de un nodo (como variables globales o settings). Sirven para alterar el comportamiento interno.
- **Acciones (Actions):** Son una mezcla de Tópicos y Servicios. Se usan para tareas **largas** (ej. "Navega hasta la cocina"). A diferencia de un servicio, una acción te envía "feedback" del progreso y puede ser cancelada a la mitad.

### 🛠️ Entregable 4 (Mini-Reto): Hackeando la Matrix
1. Ve la lista de parámetros configurables del nodo turtlesim: 
   ```bash
   ros2 param list
   ```
2. Cambia el color del fondo en vivo modificando el parámetro del rojo (`background_r`):
   ```bash
   ros2 param set /turtlesim background_r 150
   ```
   Toma una captura evidenciando el cambio de color en el simulador mediante este comando.

## 6. Explorando la Interfaz Gráfica (rqt)

### 🧠 El Concepto
Aunque la terminal es rápida y potente, escribir estructuras de datos largas o complejas (como el formato YAML de un servicio) puede ser tedioso. **rqt** es un framework gráfico oficial que consolida múltiples herramientas (plugins) de ROS 2 en una sola ventana, permitiéndote interactuar con el ecosistema a través de clics en lugar de comandos de consola.

### 🛠️ Ejercicio: Interactuar Gráficamente
Abre la ventana de `rqt` que lanzaste en el Paso 1 (o escribe `rqt` en una terminal nueva):

1. **Llamar a un Servicio (Service Caller):**
   - Ve al menú superior: `Plugins > Services > Service Caller`.
   - En el menú desplegable `Service`, selecciona `/turtle1/teleport_absolute`.
   - rqt te mostrará los campos requeridos (`x`, `y`, `theta`) ya estructurados.
   - Rellena los valores (ej. x: 5.0, y: 5.0, theta: 0.0) y presiona el botón **Call**.
   - *Verás que la tortuga salta de vuelta al centro sin haber escrito ni una sola línea de YAML en la terminal.*

2. **Modificar Parámetros (Dynamic Reconfigure):**
   - Ve a `Plugins > Configuration > Dynamic Reconfigure`.
   - Selecciona el nodo `turtlesim` en la lista de la izquierda.
   - Aparecerán deslizadores (sliders) para `background_b`, `background_g`, y `background_r`.
   - Arrastra los deslizadores y observa cómo la ventana del simulador cambia de color en tiempo real sin reiniciar nada.

### 🛠️ Entregable 5 (Mini-Reto): Pintando con rqt
Utiliza el `Service Caller` en rqt para invocar el servicio `/turtle1/set_pen`. Modifica los valores de color (`r`, `g`, `b`) a tu gusto y establece el ancho (`width`) en 5.
Luego mueve la tortuga principal desde tu terminal de teleoperación. Documenta con una captura que lograste cambiar el color y grosor de la línea dibujada.

---

## 11. Componente de Evaluación Final (Criterio RAE 1)

Para aprobar este taller y validar tu adquisición del RAE 1 (Arquitectura Distribuida y Comunicación Técnica), debes consolidar las evidencias de los **Entregables 1 al 5** y sumarle esta **Prueba Integradora** final para presentarla al docente.

**Misión Final:**
1. Inicia el simulador (`turtlesim_node`).
2. Utilizando CLI, cambia el color de fondo a un verde intenso (usando `ros2 param set` sobre `background_g`).
3. Mueve la tortuga hacia la coordenada (X:8, Y:8) llamando al servicio `/turtle1/teleport_absolute` (puedes usar la terminal o el Service Caller de rqt).
4. Abre la herramienta `rqt Node Graph` y refresca el grafo.
5. Usa el nodo de teleoperación (`turtle_teleop_key`) para conducir manualmente la tortuga de regreso al centro de la pantalla.
6. **Entregable 6 (Captura Integradora):** Toma una captura de pantalla integral de tu escritorio donde se evidencie:
   - La ventana de turtlesim con el fondo modificado y el trazo que dejó la tortuga al ser teleoperada.
   - El gráfico (Node Graph) de rqt mostrando la topología.
   - Una terminal corriendo `ros2 topic echo` capturando los datos en movimiento.
7. **Entregable 7 (Análisis Crítico):** Redacta un párrafo (máximo 5 líneas) explicando con tus propias palabras:
   *¿Cuál es la diferencia arquitectónica principal entre enviar datos por un "Topic" y hacerlo mediante un "Service"?*
8. **Entregable 8 (Aplicación al Proyecto):** Dentro del contexto de nuestro proyecto `burger_delivery`, propón y sustenta:
   - **Dos (2) casos donde usarías un Tópico:** (Ej: ¿Qué datos del robot o entorno necesitan fluir continuamente de forma asíncrona?). Sustenta por qué es la mejor opción.
   - **Dos (2) casos donde usarías un Servicio:** (Ej: ¿Qué acciones del sistema requieren que un nodo solicite algo y espere una confirmación de éxito/fracaso?). Sustenta por qué es la mejor opción.

---

## ⚠️ ¡No caigas en la trampa! (Errores Comunes)

- **Error:** Comando `ros2` no encontrado.
  - **Solución:** Olvidaste hacer el source del entorno (`source /opt/ros/jazzy/setup.bash`).
- **Error:** `ros2 topic echo` no muestra absolutamente nada.
  - **Solución:** Los tópicos solo imprimen texto en la terminal cuando hay datos viajando. Tienes que mover la tortuga desde la terminal de teleop activa.
- **Error de Sintaxis YAML:** Al usar `ros2 topic pub` o `ros2 service call`, la terminal te devuelve error de parseo.
  - **Solución:** ROS 2 usa formato YAML estricto. Revisa cuidadosamente los espacios después de los dos puntos `:`, las llaves `{}` y las comillas.

## 🏆 Resultado Esperado (Lo que ahora sabes)
Al terminar este taller, habrás validado tu conocimiento práctico sobre:
- La diferencia fundamental entre un Nodo, un Tópico, un Servicio y un Parámetro.
- El arranque y gestión de programas distribuidos de ROS 2 desde la CLI.
- El uso de herramientas de diagnóstico gráfico (`rqt`) e introspección textual.
- La capacidad de depurar y validar flujos de información, habilidades indispensables cuando algo falle en tu desarrollo de robótica física.
