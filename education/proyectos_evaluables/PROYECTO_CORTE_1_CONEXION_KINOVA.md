# Proyecto del primer corte — Conexión segura con Kinova Gen3

## 1. Descripción

La persona participante debe diseñar e implementar un package ROS 2 llamado `burger_kinova_connection` que permita verificar, monitorear y utilizar de forma controlada la comunicación entre el proyecto `burger_delivery` y un manipulador Kinova Gen3 de 7 grados de libertad.

El package utilizará el stack existente `ros2_kortex`. No debe reimplementar el driver, modificar el protocolo propietario de Kinova ni comunicarse directamente con la API Kortex. Su responsabilidad comienza en las interfaces ROS 2 que publica el driver.

Esta especificación define requisitos técnicos y pruebas de aceptación. No es una rúbrica de calificación.

## 2. Objetivo

Construir una arquitectura ROS 2 reproducible que pueda:

1. Iniciar o descubrir el driver del Kinova.
2. Confirmar que el robot y sus controladores están disponibles.
3. Recibir y validar el estado de las siete articulaciones.
4. Publicar un diagnóstico comprensible del enlace.
5. Enviar una trayectoria articular de prueba bajo condiciones explícitas de seguridad.
6. Funcionar tanto en una estación única como en una arquitectura DDS distribuida entre dos computadores.

## 3. Plataforma de referencia

| Componente | Referencia del proyecto |
|---|---|
| Sistema operativo | Ubuntu 24.04 |
| Middleware | ROS 2 Jazzy |
| Manipulador | Kinova Gen3, 7 DOF |
| Pinza disponible | Robotiq 2F-85 |
| Driver | `kortex_driver` mediante `ros2_control` |
| Bringup | `kortex_bringup/gen3.launch.py` |
| Control articular | `joint_trajectory_controller` |
| Estado articular | `/joint_states` |
| Red | Ethernet hacia el robot y DDS entre estaciones ROS 2 |

La dirección `192.168.1.10` documentada en el entorno local puede utilizarse como ejemplo de laboratorio, pero debe suministrarse mediante configuración o argumento de launch. No debe quedar escrita como constante dentro del código Python.

## 4. Límite de responsabilidades

### 4.1. Stack Kortex

`kortex_driver` es responsable de:

- La sesión TCP/UDP con el robot físico.
- La lectura y escritura de las interfaces de hardware.
- La integración con `controller_manager` y `ros2_control`.
- El estado articular, la pinza y la gestión interna de fallas.

### 4.2. Package del proyecto

`burger_kinova_connection` es responsable de:

- Configurar o descubrir el grafo ROS 2.
- Verificar disponibilidad y estado de los controladores.
- Interpretar `/joint_states`.
- Detectar pérdida o degradación de la comunicación.
- Publicar diagnósticos.
- Construir y enviar una meta articular segura mediante la acción estándar del controlador.

### 4.3. Fuera del alcance

No forman parte de este proyecto:

- Planeación cartesiana o cinemática inversa con MoveIt 2.
- Pick-and-place.
- Percepción con cámaras o AprilTags.
- Modelado completo de la celda mediante URDF/XACRO.
- Modificación de `ros2_kortex` o de sus controladores.
- Comandos directos a la API propietaria de Kinova.
- Recuperación automática de fallas o activación remota de una parada de emergencia.

## 5. Arquitectura

```mermaid
flowchart LR
    R[Kinova Gen3] <-->|TCP/UDP Kortex| D[kortex_driver + ros2_control]
    D -->|sensor_msgs/JointState| J[/joint_states]
    D --> C[controller_manager]
    C --> A[FollowJointTrajectory]
    J --> M[kinova_monitor]
    C --> M
    M --> X[/burger/kinova/diagnostics]
    U[safe_trajectory_client] --> A
    P[Operador] -->|habilitación explícita| U
```

En despliegue distribuido, el driver se ejecuta en la estación conectada físicamente al Kinova y los nodos del proyecto pueden ejecutarse en una segunda estación. DDS comunica las estaciones; DDS no sustituye la conexión TCP/UDP entre el driver y el robot.

## 6. Estructura obligatoria

El package debe ubicarse en:

```text
~/ros2_ws/src/burger_delivery/burger_kinova_connection/
```

Estructura mínima:

```text
burger_kinova_connection/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── burger_kinova_connection
├── burger_kinova_connection/
│   ├── __init__.py
│   ├── kinova_monitor.py
│   └── safe_trajectory_client.py
├── launch/
│   └── kinova_connection.launch.py
├── config/
│   └── kinova_connection.yaml
├── test/
└── README.md
```

Se recomienda `ament_python` para concentrar el proyecto en los conceptos de comunicación ROS 2. Los tipos de mensaje, servicio y acción deben provenir de interfaces estándar existentes.

## 7. Dependencias mínimas

El `package.xml` debe declarar solamente las dependencias utilizadas. Como mínimo se esperan:

- `rclpy`
- `sensor_msgs`
- `diagnostic_msgs`
- `control_msgs`
- `trajectory_msgs`
- `controller_manager_msgs`
- `launch`
- `launch_ros`
- `kortex_bringup`

No se debe declarar MoveIt 2 como dependencia de este package.

## 8. Interfaces ROS 2

### 8.1. Interfaces consumidas

| Nombre | Tipo | Uso |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | Posición, velocidad y nombres de las articulaciones |
| `/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers` | Confirmar controladores disponibles y activos |
| `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Enviar y supervisar la trayectoria de prueba |

### 8.2. Interfaz publicada

| Nombre | Tipo | Uso |
|---|---|---|
| `/burger/kinova/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Estado consolidado de conexión, telemetría y controladores |

El tópico de diagnóstico debe usar comunicación fiable. La suscripción a `/joint_states` debe emplear un perfil compatible con datos de sensores y documentar la decisión de QoS.

## 9. Parámetros

Los siguientes parámetros deben residir en `config/kinova_connection.yaml` y poder sobrescribirse desde launch:

| Parámetro | Tipo | Valor seguro inicial | Propósito |
|---|---|---:|---|
| `start_driver` | `bool` | `false` | Incluir o no `kortex_bringup` en la estación actual |
| `robot_ip` | `string` | `0.0.0.0` en modo fake | Dirección del robot cuando se inicia el driver localmente |
| `use_fake_hardware` | `bool` | `true` | Validar el package sin movimiento físico |
| `launch_rviz` | `bool` | `false` | Abrir RViz como herramienta de observación |
| `enable_motion` | `bool` | `false` | Habilitación explícita de comandos físicos |
| `joint_state_timeout_s` | `double` | `1.0` | Tiempo máximo sin recibir `/joint_states` |
| `min_joint_state_hz` | `double` | `20.0` | Frecuencia mínima aceptada para la prueba |
| `max_joint_delta_rad` | `double` | `0.10` | Cambio máximo permitido por articulación |
| `trajectory_duration_s` | `double` | `5.0` | Duración mínima de la trayectoria de prueba |
| `diagnostic_rate_hz` | `double` | `1.0` | Frecuencia de publicación del diagnóstico |
| `safe_joint_positions_rad` | `double[7]` | Definido y aprobado en laboratorio | Meta articular de la prueba controlada |
| `joint_min_rad` / `joint_max_rad` | `double[7]` | Límites aprobados del modelo real | Validación local previa al envío de la meta |

Los parámetros de seguridad deben validarse al iniciar. Un valor ausente, inválido o fuera de rango debe impedir el movimiento y producir un mensaje de error claro. Si `start_driver:=true` y `use_fake_hardware:=false`, el valor `0.0.0.0` debe rechazarse y exigirse una IP real.

## 10. Requisitos funcionales

### RF-01. Construcción e instalación

El package debe compilar con:

```bash
cd ~/ros2_ws
colcon build --packages-select burger_kinova_connection --symlink-install
source install/setup.bash
```

Los ejecutables y el launch deben quedar disponibles desde `install/`; no se aceptan ejecuciones que dependan de rutas directas dentro de `src/`.

### RF-02. Launch unificado

`kinova_connection.launch.py` debe:

- Cargar `kinova_connection.yaml`.
- Iniciar `kinova_monitor`.
- Incluir opcionalmente `kortex_bringup/gen3.launch.py` cuando `start_driver:=true`.
- Transferir `robot_ip`, `use_fake_hardware`, `launch_rviz`, `dof:=7` y la configuración de la pinza al bringup.
- Permitir `start_driver:=false` para operar como cliente de un driver ejecutado en otra estación.

### RF-03. Monitoreo del estado articular

`kinova_monitor` debe:

- Suscribirse a `/joint_states`.
- Comprobar la presencia de `joint_1` a `joint_7` sin depender del orden del arreglo.
- Rechazar mensajes cuyos arreglos de nombres y posiciones sean incoherentes.
- Medir frecuencia aproximada, edad del último mensaje y cantidad de interrupciones.
- Distinguir entre conexión saludable, degradada y perdida.

### RF-04. Estado de controladores

El monitor debe consultar `/controller_manager/list_controllers` y comprobar como mínimo:

- `joint_state_broadcaster` en estado `active`.
- `joint_trajectory_controller` en estado `active` antes de habilitar movimiento.

La indisponibilidad del servicio o un controlador inactivo debe aparecer en el diagnóstico; no debe provocar la terminación inesperada del nodo.

### RF-05. Diagnóstico consolidado

`/burger/kinova/diagnostics` debe informar como mínimo:

- Estado general: `OK`, `WARN` o `ERROR`.
- Edad y frecuencia estimada de `/joint_states`.
- Articulaciones detectadas y faltantes.
- Estado del controlador de trayectoria.
- Modo `fake` o hardware real.
- Habilitación o bloqueo de movimiento.
- Último error observado y acción recomendada.

### RF-06. Cliente de trayectoria segura

`safe_trajectory_client` debe:

- Usar `ActionClient` con `control_msgs/action/FollowJointTrajectory`.
- Esperar explícitamente la disponibilidad del servidor de acción.
- Leer el estado articular actual antes de construir la meta.
- Cargar una pose de prueba previamente aprobada desde YAML.
- Verificar cantidad y nombres de articulaciones.
- Verificar límites configurados y `max_joint_delta_rad` para cada articulación.
- Exigir `enable_motion:=true` para enviar una meta al hardware real.
- Mostrar un resumen de la trayectoria y solicitar confirmación del operador cuando se ejecute de forma interactiva.
- Reportar meta aceptada o rechazada, feedback, resultado y código de error.
- Cancelar o finalizar de forma controlada si vence el tiempo de espera.

El cliente debe poder ejecutar toda la validación en modo seco sin enviar la meta.

### RF-07. Operación distribuida

El sistema debe admitir esta distribución:

```text
Estación A: conexión Ethernet al Kinova + kortex_bringup
Estación B: burger_kinova_connection con start_driver:=false
Ambas: misma red y ROS_DOMAIN_ID acordado
```

La documentación debe identificar qué tráfico corresponde a Kortex TCP/UDP y cuál corresponde al descubrimiento y transporte DDS entre nodos ROS 2.

### RF-08. Manejo de fallas

El sistema debe manejar sin cerrarse abruptamente:

- IP inalcanzable.
- Ausencia del driver.
- Pérdida temporal de `/joint_states`.
- Servicio de controladores no disponible.
- Controlador inactivo.
- Servidor de acción ausente.
- Meta rechazada o abortada.
- Mensaje articular incompleto.
- Desconexión de una estación DDS.

## 11. Requisitos de seguridad

1. `enable_motion` debe iniciar siempre en `false`.
2. La primera validación debe realizarse con `use_fake_hardware:=true`.
3. Toda prueba física requiere espacio despejado, parada de emergencia accesible y autorización del responsable del laboratorio.
4. La velocidad y aceleración deben permanecer limitadas por la configuración aprobada del controlador.
5. No se permite desactivar límites articulares ni protecciones del fabricante.
6. No se permite limpiar fallas automáticamente para continuar una prueba.
7. Si el estado articular está vencido, incompleto o fuera de límites, el cliente debe bloquear el envío de metas.
8. Después de una pérdida de comunicación, el movimiento debe permanecer deshabilitado hasta una nueva habilitación explícita.

## 12. Requisitos de calidad

- No almacenar IP, usuario o contraseña como constantes en el código.
- No usar rutas absolutas dependientes del computador del desarrollador.
- No modificar archivos dentro de `ros2_kortex`.
- Mantener nombres de nodos, tópicos y parámetros en `snake_case`.
- Evitar bloques `except` vacíos y registrar la causa de cada error.
- Incluir pruebas unitarias para validación de mensajes, timeouts y límites de metas.
- Superar las pruebas de estilo configuradas para paquetes `ament_python`.
- Mantener fuera del repositorio archivos de log grandes, bolsas ROS y credenciales.

## 13. Modos de ejecución

### 13.1. Validación sin robot

```bash
ros2 launch burger_kinova_connection kinova_connection.launch.py \
  start_driver:=true \
  robot_ip:=0.0.0.0 \
  use_fake_hardware:=true \
  enable_motion:=false
```

### 13.2. Driver y monitor en la misma estación

```bash
ros2 launch burger_kinova_connection kinova_connection.launch.py \
  start_driver:=true \
  robot_ip:=192.168.1.10 \
  use_fake_hardware:=false \
  enable_motion:=false
```

La IP anterior es un ejemplo del laboratorio y debe ajustarse a la configuración verificada el día de la práctica.

### 13.3. Monitor en una segunda estación

```bash
export ROS_DOMAIN_ID=<dominio_del_equipo>
ros2 launch burger_kinova_connection kinova_connection.launch.py \
  start_driver:=false \
  enable_motion:=false
```

## 14. Pruebas de aceptación

| ID | Prueba | Procedimiento mínimo | Resultado esperado |
|---|---|---|---|
| PA-01 | Compilación limpia | Compilar únicamente el package y ejecutar `colcon test` | Sin errores de compilación, importación o estilo |
| PA-02 | Grafo en modo fake | Lanzar con hardware simulado e inspeccionar nodos e interfaces | Monitor activo, siete articulaciones y diagnóstico publicado |
| PA-03 | Telemetría real | Conectar el Kinova y observar `/joint_states` durante al menos 60 s | Siete articulaciones, sin interrupciones y frecuencia superior al mínimo configurado |
| PA-04 | Controladores | Consultar `list_controllers` desde el nodo y desde CLI | Broadcaster y controlador de trayectoria activos |
| PA-05 | Pérdida de enlace | Detener el driver o aislar el cliente de la red | Transición a `ERROR` después del timeout, sin caída del monitor |
| PA-06 | Recuperación | Restaurar el driver o la red | Regreso a estado saludable sin reiniciar el monitor |
| PA-07 | Validación de meta | Probar pose incompleta, límite excedido y movimiento deshabilitado | Las tres metas se bloquean antes de contactar el action server |
| PA-08 | Movimiento autorizado | Ejecutar la pose aprobada con supervisión | Meta aceptada, movimiento lento y resultado exitoso |
| PA-09 | DDS distribuido | Driver en estación A y monitor en estación B | Descubrimiento, telemetría y diagnóstico funcionales entre estaciones |
| PA-10 | Reproducibilidad | Seguir el README desde un workspace limpio | Otra persona puede compilar, lanzar y repetir las pruebas |

## 15. Entregables

1. Package completo `burger_kinova_connection` dentro del repositorio `burger_delivery`.
2. `README.md` del package con instalación, arquitectura, interfaces, parámetros y comandos de ejecución.
3. `config/kinova_connection.yaml` sin credenciales y con movimiento deshabilitado por defecto.
4. `docs/VALIDACION_CORTE_1.md` con resultados de PA-01 a PA-10, comandos utilizados, fecha, entorno y observaciones.
5. Diagrama del grafo ROS 2 y distribución entre estaciones.
6. Registro breve de frecuencia, pérdida y recuperación de `/joint_states`.
7. Demostración funcional en hardware simulado y en el Kinova real.

Los resultados deben poder verificarse mediante texto, salidas de comandos y registros. Las capturas pueden complementar la evidencia, pero no reemplazan una descripción reproducible.

## 16. Contenido mínimo del README del package

El README entregado debe permitir que una persona que no desarrolló el proyecto pueda:

1. Identificar las dependencias.
2. Compilar el package.
3. Ejecutarlo en modo fake.
4. Conectarlo al robot real sin editar el código.
5. Ejecutarlo como cliente desde una segunda estación.
6. Interpretar `/burger/kinova/diagnostics`.
7. Ejecutar de manera segura la prueba articular.
8. Diagnosticar una falla de red, controlador o action server.

## 17. Definición de terminado

El proyecto se considera técnicamente completo cuando:

- Existe un package instalable y no una colección de scripts sueltos.
- El monitor distingue correctamente conexión saludable, degradada y perdida.
- La información de las siete articulaciones es verificable.
- El estado de los controladores se consulta mediante ROS 2.
- Ninguna meta puede enviarse con movimiento deshabilitado o telemetría inválida.
- La trayectoria aprobada se ejecuta y reporta su resultado.
- El cliente funciona desde una segunda estación mediante DDS.
- Las diez pruebas de aceptación están documentadas y son reproducibles.

## 18. Referencias del repositorio

- [Syllabus oficial de la asignatura](../syllabus/Syllabus_Robotica_ROS2_Experimental.docx).
- [Instalación y pruebas del stack Kortex](../../ros2_setup/INSTALACION_KORTEX.md).
- [Diagnóstico de red ROS 2](../../network_setup/DIAGNOSTICO_RED.md).
- [Configuración de red ROS 2](../../network_setup/ROS2_NETWORK_CONFIG.md).
