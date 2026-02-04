# Guía de Verificación de Instalación ROS 2 Jazzy en WSL

Esta guía te ayudará a verificar que **ROS 2 Jazzy Jalisco** esté correctamente instalado en tu entorno de **WSL (Windows Subsystem for Linux)** usando las herramientas de Antigravity.

---

## 1. Cómo abrir la terminal de Ubuntu en Antigravity

Para ejecutar comandos de ROS 2 directamente desde **Antigravity** en tu entorno de Ubuntu:

1.  **Abrir el selector de terminales:** En el panel de la terminal de Antigravity, busca el botón para abrir una nueva terminal (icono `+` o flecha hacia abajo).
2.  **Seleccionar Ubuntu:** Selecciona **"Ubuntu"** (o tu distribución de WSL) de la lista de perfiles disponibles. Esto te abrirá una terminal nativa de Linux directamente en el IDE.
3.  **Verificar que estás en Linux:** Deberías ver un prompt similar a `usuario@nombre-pc:~$`.

*Si no encuentras el perfil de Ubuntu, puedes abrir la terminal por defecto (PowerShell) y simplemente escribir:*
```bash
wsl ~ -d Ubuntu
```
*(El `~` asegura que inicies en tu carpeta de usuario de Linux).*

---

## 2. Pasos de Verificación

### Paso A: Verificar la versión de Ubuntu
ROS 2 Jazzy requiere **Ubuntu 24.04 (Noble Numbat)**. Ejecuta:
```bash
lsb_release -a
```
Deberías ver `Description: Ubuntu 24.04 LTS`.

### Paso B: Configurar el entorno (Sourcing)
Antes de usar ROS 2, debes cargar sus variables de entorno. Ejecuta:
```bash
source /opt/ros/jazzy/setup.bash
```
*Tip: Para no tener que escribir esto cada vez, agrégalo a tu archivo `.bashrc`:*
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### Paso C: Verificar variables y ruta
Una vez hecho el `source`, el sistema debe reconocer dónde está ROS 2.

1. **Verificar ruta del ejecutable:**
   ```bash
   which ros2
   ```
   Debería devolver: `/opt/ros/jazzy/bin/ros2`. *Si no devuelve nada, el Paso B falló.*

2. **Verificar variables de entorno:**
   ```bash
   printenv | grep ROS
   ```
   Deberías ver: `ROS_DISTRO=jazzy`, `ROS_VERSION=2`, etc.

#### Nota sobre Advertencias de Descubrimiento (Discovery)
Es común ver este mensaje en ROS 2 Jazzy:
> `[WARN] [rcl]: ROS_LOCALHOST_ONLY is deprecated... Use ROS_AUTOMATIC_DISCOVERY_RANGE and ROS_STATIC_PEERS instead.`

**¿Qué significa?**
En Jazzy, la variable `ROS_LOCALHOST_ONLY` ha sido reemplazada por un sistema más flexible (`ROS_AUTOMATIC_DISCOVERY_RANGE`). Si ves este aviso, tu instalación es correcta y el sistema simplemente aplica las nuevas reglas por defecto.

### Paso D: Verificar comandos básicos
Y verifica el estado general con:
```bash
ros2 doctor
```

### Paso E: Prueba de comunicación (Demo Talker-Listener)
Esta es la prueba definitiva para asegurar que los nodos pueden comunicarse.

1.  Abre una terminal y ejecuta el **Talker** (Publicador):
    ```bash
    source /opt/ros/jazzy/setup.bash
    ros2 run demo_nodes_cpp talker
    ```
2.  Abre una **segunda terminal** (ver sección 1) y ejecuta el **Listener** (Suscriptor):
    ```bash
    source /opt/ros/jazzy/setup.bash
    ros2 run demo_nodes_py listener
    ```

Si ves al "Talker" enviando mensajes y al "Listener" recibiéndolos, ¡tu instalación es exitosa!

### Paso F: Prueba de Interfaz Gráfica (rqt_graph)
Esta prueba verifica que las aplicaciones gráficas de ROS 2 funcionen correctamente desde WSL hacia Windows (requiere WSLg).

1.  Con el **Talker** y el **Listener** aún corriendo (del Paso E), abre una **tercera terminal**.
2.  Ejecuta el visualizador de la arquitectura:
    ```bash
    source /opt/ros/jazzy/setup.bash
    rqt_graph
    ```
3.  Debería abrirse una ventana en Windows mostrando los dos nodos (`/talker` y `/listener`) y la conexión a través del tópico `/chatter`.

*Si la ventana no se abre, verifica que tienes **WSLg** instalado o un servidor X configurado.*

---

## 3. Tutoriales Oficiales y Recursos

Aquí tienes los enlaces clave para seguir aprendiendo:

*   **[Documentación Oficial de ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)** - El punto de partida de todo.
*   **[Tutoriales de ROS 2 (Nivel Principiante)](https://docs.ros.org/en/jazzy/Tutorials.html)** - Guías paso a paso para aprender CLI y conceptos.
*   **[Guía de Instalación en Ubuntu](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)** - Por si necesitas revisar algún paso previo.

---

---

## 4. WSL vs. Instalación Nativa en Windows

¿Por qué usar ROS 2 en WSL en lugar de instalarlo directamente en Windows? Aquí tienes una comparativa:

### Ventajas de usar WSL2
*   **Compatibilidad de Paquetes:** La gran mayoría de los paquetes de ROS 2 se desarrollan y prueban primero en Linux. En WSL tienes acceso al ecosistema completo sin errores de compilación comunes en Windows.
*   **Rendimiento:** WSL2 utiliza un kernel Linux real, lo que ofrece un rendimiento superior en procesos de compilación y ejecución de nodos en comparación con la capa de abstracción de Windows.
*   **Herramientas Gráficas (WSLg):** Gracias a WSLg, herramientas como Rviz2 o Gazebo funcionan con aceleración por hardware casi de forma nativa en Windows 11.
*   **Aislamiento:** Mantienes tu sistema Windows limpio de dependencias complejas de Python y C++ que ROS 2 requiere.

### Desventajas / Desafíos
*   **Acceso a Hardware:** Conectar sensores físicos (Lidar, Cámaras, Joysticks) por USB puede ser más complejo y requiere herramientas adicionales como `usbipd-win`.
*   **Redes:** WSL2 utiliza una red virtualizada. Si necesitas que tu PC se comunique con un robot físico en la misma red WiFi, podrías necesitar configurar el modo "mirrored" en el archivo `.wslconfig`.
*   **Memoria:** WSL2 puede consumir mucha RAM si no se limita en su configuración, ya que reserva recursos para la máquina virtual.

> **Veredicto:** Para aprendizaje, simulación y desarrollo de software, **WSL2 es la mejor opción en Windows**. Solo se recomienda la instalación nativa si el proyecto depende estrictamente de librerías exclusivas de Windows o drivers de hardware que no funcionan en WSL.

---

## 5. Ventajas de usar la extensión "Open Remote - WSL"

Ahora que has instalado la extensión **Open Remote - WSL** desde Open VSX, tu flujo de trabajo con ROS 2 Jazzy mejora significativamente:

*   **Intellisense Nativo:** El editor ahora puede "ver" las librerías de ROS instaladas en `/opt/ros/jazzy`. Esto habilita el autocompletado inteligente, resaltado de errores y navegación al código fuente (F12) para tus proyectos de C++ y Python.
*   **Terminal Integrada por Defecto:** Al abrir una carpeta a través de la extensión, la terminal de Antigravity se abrirá automáticamente en el entorno de Ubuntu, ahorrándote el comando `wsl` manual.
*   **Depuración (Debugging):** Puedes usar el depurador de Antigravity para poner puntos de interrupción (*breakpoints*) en tus nodos de ROS 2 y ejecutarlos directamente dentro del entorno de Linux.
*   **Gestión de Archivos Transparente:** No necesitas preocuparte por las rutas de Windows (`C:\...`) vs Linux (`/home/...`). La extensión maneja la traducción de rutas de forma invisible, evitando problemas de permisos.
*   **Rendimiento de Git:** Al trabajar "dentro" de la distribución, las operaciones de Git son mucho más rápidas que si intentaras manejar archivos de Linux desde el explorador de Windows.

---

### Notas finales y Recursos
*   **Interfaz Gráfica:** Si necesitas usar herramientas como `Rviz2` o `Gazebo`, asegúrate de tener instalado **WSLg** (incluido en Windows 10/11 actualizado).
*   **Red:** WSL2 maneja su propia red. Si tienes problemas de comunicación entre Windows y WSL, revisa el firewall.
