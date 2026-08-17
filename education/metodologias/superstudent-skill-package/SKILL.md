---
name: superstudent-robotics-methodology
description: Diagnose and document collaborative-robotics integration problems using lessons from the Burger Delivery experiment with ROS 2, Kinova Gen3, TurtleBots, AprilTag, TF trees, URDF, MoveIt 2, perception, networking, and laboratory troubleshooting. Use when an agent must inspect a robotics repository, isolate a failure by layer, distinguish model, runtime and calibration faults, or turn a resolved laboratory problem into reusable technical guidance.
---
# 🎓 SuperStudent: A Troubleshooting and Methodology Skill for Collaborative Robotics Education

> **Experimento:** Burger Delivery (UMNG) — Kinova Gen3 + TurtleBots + AprilTag + MoveIt 2  
> **Última actualización:** 2026-05-14  
> **Estado:** Activa — crece con cada problema resuelto en el experimento

---

## Qué es SuperStudent

SuperStudent captura la **experiencia y experticia acumulada** durante el desarrollo del experimento Burger Delivery. No es un manual ni un tutorial — es el conocimiento destilado de **qué técnicas y metodologías resultaron más efectivas** para avanzar cuando las cosas no funcionaban.

Cada entrada fue validada resolviendo un problema real en un laboratorio real. Si estás enfrentando un proyecto de robótica colaborativa similar, SuperStudent te transfiere la experiencia de quienes ya pasaron por ahí.

**Cómo crece:** Cada vez que se resuelve un problema no trivial en el experimento, se agrega una entrada al [Log de Aprendizaje](#parte-2-log-de-aprendizaje). La skill se enriquece orgánicamente — no con teoría, sino con experiencia probada.

---

## Parte 1: Técnicas que Aceleraron el Desarrollo

### 1.1 Diseñar el árbol TF completo ANTES de escribir URDFs

**Qué hicimos:** Antes de crear cualquier archivo URDF, dibujamos en papel (y luego en SVG) la jerarquía completa de frames: desde `map` hasta `burger_grip_frame`, pasando por los carritos y la cámara.

**Por qué funcionó:** Cada vez que alguien saltó a escribir URDF sin tener claro el árbol, terminó con frames huérfanos, loops, o dual-publishers que rompían todo. Cuando el árbol estaba claro primero, la implementación era mecánica.

**La regla que nos salvó:** *"Calibra de la raíz hacia las hojas, nunca al revés."* Si `tag_mesa` está desplazado 5cm, corregirlo ahí — no compensar moviendo `tag_carrito`.

**Aplicable cuando:** Arrancas cualquier sistema multi-robot o multi-frame en ROS 2.

---

### 1.2 Separar URDFs por "fuente de verdad" de su posición

**Qué hicimos:** En lugar de un mega-URDF con todo, separamos en:
- `delivery_scene_fixed.urdf` → la escena fija (mesa, Kinova, zonas de staging)
- `car1_apriltag.urdf` / `car2_apriltag.urdf` → cada carrito móvil por separado

Cada uno con su propio `robot_state_publisher` y namespace de remapping.

**Por qué funcionó:** Los carritos se mueven — su posición viene de la cámara, no del URDF. Si estaban "soldados" a la escena fija, era imposible actualizar su posición dinámicamente. Al separarlos, el nodo de visión publica `tag_mesa → tag_carrito` y el URDF del carrito cuelga de ahí.

**El error que evitamos:** Meter `car1_base_link` como link fijo de la escena. Al principio parecía más fácil, pero hizo imposible la localización dinámica.

**Aplicable cuando:** Tienes objetos cuya posición viene de un sensor (cámara, LiDAR, GPS) en vez de estar predefinida.

---

### 1.3 Cancelación de perspectiva para localización eye-in-hand

**Qué hicimos:** En lugar de publicar la posición absoluta de cada tag detectado por la cámara del Kinova (que vibraba y acumulaba error cinemático), publicamos la **transformación relativa entre dos tags**:

```
T(tag_mesa → tag_carrito) = T(cam → tag_mesa)⁻¹ × T(cam → tag_carrito)
```

**Por qué funcionó:** La cámara ve ambos tags desde la misma posición, en el mismo instante. Al invertir una observación y multiplicar por la otra, **toda la vibración del brazo se cancela**. El carrito quedaba "anclado" sólidamente a la mesa, sin importar cómo se moviera el brazo.

**La intuición clave:** La cámara se convierte en un observador neutral. No importa dónde esté — importa que ve dos cosas a la vez y calcula la relación entre ellas.

**Requisito:** Necesitas un **tag de referencia fijo** cuya posición real coincida con lo que dice el URDF (el joint `table_to_tag_mesa`). Si ese tag está mal calibrado, todo el sistema tiene un error sistemático.

**Aplicable cuando:** Tienes cámara montada en un efector que se mueve, y fiduciales de referencia en la escena.

---

### 1.4 Automatizar el parcheo de drivers industriales

**Qué hicimos:** Los drivers del Kinova (`ros2_kortex`) traían timeouts UDP de 500ms-1000ms y parámetros de simulación residuales que causaban jittering. Creamos `apply_kinova_smooth_movement.py` — un script que:
- Reduce el timeout del router UDP a 200ms
- Activa el bus interno del gripper
- Limpia parámetros de simulación (`sim_gazebo`, `sim_isaac`)

**Por qué funcionó:** En vez de que cada estudiante hiciera parches manuales (que se perdían al reinstalar), el script era idempotente y reproducible. Un `python3 apply_kinova_smooth_movement.py` + `colcon build` y listo.

**La filosofía:** *Nunca parches manuales. Siempre scripts. Si tienes que explicar un parche más de una vez, automatízalo.*

**Otra técnica que funcionó para MoveIt:** Comenzar con 5% de velocidad y aceleración (`max_velocity_scaling_factor = 0.05`). Subir gradualmente. Los defaults (100%) causan arranques violentos que rompen cosas.

**Aplicable cuando:** Integras cualquier manipulador industrial cuyo driver ROS 2 tiene defaults conservadores.

---

### 1.5 Modos de operación switchable via launch arguments

**Qué hicimos:** El hardware completo casi nunca está disponible. Creamos un switch `use_static_carts` en el launch:
- **False (default):** Los carritos esperan TFs del nodo de localización real. Si no hay cámara, simplemente no aparecen.
- **True:** Se publican TFs estáticos hardcodeados para visualizar todo en RViz sin necesidad de cámara.

**Por qué funcionó:** Los equipos de URDF/MoveIt podían trabajar sin depender de que la cámara estuviera funcionando, y viceversa.

**El error más frecuente:** Lanzar `use_static_carts:=true` al mismo tiempo que el nodo de AprilTag → dos publicadores del mismo TF → árbol ambiguo → todo roto. **Regla: nunca mezcles modos.**

**Aplicable cuando:** Tu sistema tiene subsistemas que no siempre están disponibles simultáneamente.

---

### 1.6 Debug por capas: la secuencia que ahorra horas

**Qué hicimos:** Establecimos un protocolo fijo de diagnóstico. Ante cualquier fallo, preguntar en este orden:

1. **¿Es sintaxis?** → `xmllint --noout`, `check_urdf`, `python3 -m py_compile`
2. **¿Es TF?** → `ros2 run tf2_tools view_frames`, `tf2_echo`
3. **¿Es red?** → `ros2 topic list`, `ros2 topic hz`, diagnóstico WiFi
4. **¿Es lógica?** → Logs del nodo, estado del action server

**Por qué funcionó:** Los estudiantes dejaron de saltar al código ante el primer error. El 70% de los problemas se resolvían en las capas 1 o 2 sin tocar una sola línea de código.

**Anti-patrón clásico:** "El carrito no aparece en RViz → voy a modificar el URDF del carrito." Respuesta correcta: verificar si el TF `tag_mesa → tag_carrito` se está publicando.

**Aplicable cuando:** Siempre. Es universal.

---

### 1.7 Definir contratos de integración antes del código

**Qué hicimos:** Antes de que los equipos (Kinova vs Carrito) escribieran código, se definió:
- **Contrato TF:** Qué frames publica cada equipo, punto de unión (`tag_mesa`)
- **Contrato de servicios:** Interface exacta de `/car1/prepare_delivery_pose`, condiciones de `success`
- **Regla de oro:** "Nada hardcodeado — todo se lee dinámicamente del TF tree"

**Por qué funcionó:** La integración final fue cuestión de lanzar ambos subsistemas. Sin el contrato, los equipos habrían hecho suposiciones incompatibles sobre frames, coordenadas, y protocolos.

**La frase que guió todo:** *"Los sistemas no fallan en sus partes, sino en las interfaces que los unen."*

**Aplicable cuando:** Hay más de una persona trabajando en el sistema.

---

### 1.8 Red WiFi: comprimir antes de transmitir, siempre

**Qué hicimos:** En un laboratorio con 15+ estaciones ROS 2 en la misma WiFi:
- Nunca publicar `sensor_msgs/Image` crudo — siempre `/compressed`
- QoS: `Best Effort` para video (no reintentar paquetes perdidos)
- Limitar a 10-15 FPS desde el driver de cámara
- Verificar con `ros2 topic delay` la latencia end-to-end
- Usar `DOMAIN_ID` único para cada grupo de trabajo

**La técnica de detección de conflictos:** `ss -ulnp | grep 74` para ver puertos DDS activos y calcular qué `DOMAIN_ID` están usando otros equipos en la misma red.

**Aplicable cuando:** Cualquier sistema ROS 2 que transmita imágenes o pointclouds por WiFi.

---

### 1.9 Vendorizar meshes y descripciones de terceros

**Qué hicimos:** Copiamos los meshes del Kinova y Robotiq dentro de `vendor/` en nuestro repositorio, en vez de depender de la instalación global de `kortex_description`.

**Por qué funcionó:** `git clone` + `colcon build` = sistema funcional. Sin pasos manuales extra de "instala este paquete en esta versión específica".

**La regla:** Copiar solo los recursos estáticos (meshes, configs), no el código fuente del driver. El driver se instala aparte, los recursos viajan con tu repo.

**Aplicable cuando:** Dependes de recursos de terceros y quieres reproducibilidad.

---

### 1.10 AI multimodal como percepción zero-shot

**Qué hicimos:** Usamos Gemini Robotics-ER como "cerebro visual" en vez de YOLO:
- La cámara del robot captura una imagen
- Se envía a la API con un prompt en lenguaje natural: *"Encuentra la caja de hamburguesa que está libre y segura de agarrar"*
- Retorna coordenadas 2D normalizadas [Y, X]
- Se proyecta a 3D usando la matriz intrínseca + profundidad

**Por qué funcionó:** Cambiar de "caja de hamburguesa" a "vaso de café" = cambiar una línea de prompt. Con YOLO habría requerido recolectar y etiquetar un dataset nuevo.

**Regla de integración:** La llamada a la API **siempre** en un hilo separado. Si bloqueas el `spin()` del nodo, todo se congela.

**Aplicable cuando:** Los objetos cambian frecuentemente o necesitas razonamiento semántico además de detección.

---

## Parte 2: Log de Aprendizaje (se actualiza con cada descubrimiento)

> Cada vez que resolvemos un problema no trivial, se agrega una entrada aquí con fecha, problema, solución, y la lección extraída. Este log es el mecanismo por el cual la skill crece.

### Formato de entrada:
```
### [FECHA] — Título corto del problema
- **Problema:** Qué pasó
- **Causa raíz:** Por qué pasó
- **Solución:** Qué hicimos
- **Lección:** Qué generalizamos para el futuro
- **Archivos involucrados:** paths relevantes
```

---

### [2026-04] — Jittering del Kinova al mover brazo + gripper simultáneamente
- **Problema:** El brazo vibraba violentamente cuando MoveIt enviaba trayectorias mientras el gripper se estaba cerrando.
- **Causa raíz:** El gripper usaba un canal de comunicación UDP separado al brazo (`use_internal_bus_gripper_comm=false` por default), causando desincronización.
- **Solución:** Script `apply_kinova_smooth_movement.py` que inyecta `use_internal_bus_gripper_comm=true` en los XACRO del driver.
- **Lección:** Los defaults de los fabricantes priorizan compatibilidad, no rendimiento. Siempre revisar la topología de comunicación interna.
- **Archivos:** `scripts/apply_kinova_smooth_movement.py`, `docs/manipulation/MEJORAS_MOVIMIENTO_KINOVA.md`

---

### [2026-04] — Carritos "invisibles" en RViz en modo producción
- **Problema:** Al lanzar sin `use_static_carts:=true`, los carritos no aparecían aunque sus URDFs eran correctos.
- **Causa raíz:** No era un error — era el comportamiento esperado. Sin nodo de localización AprilTag publicando `tag_mesa → tag_carrito`, no hay ruta en el TF tree de `map` a `car1_base_link`, y RViz no puede renderizar lo que no puede ubicar.
- **Solución:** Documentar que esto es normal y que la ausencia es diagnóstica: si no aparecen, significa que la cámara/localización no están corriendo.
- **Lección:** "No aparece" no siempre es un error. A veces es información: te dice qué subsistema falta.
- **Archivos:** `burger_description/launch/display.launch.py`, `vision_setup/LOCALIZACION_APRILTAG.md`

---

### [2026-04] — Doble publicación del TF de carritos rompía el árbol
- **Problema:** Un estudiante lanzó `use_static_carts:=true` y simultáneamente inició el nodo de AprilTag. Los carritos saltaban entre dos posiciones erráticamente.
- **Causa raíz:** Dos nodos publicaban `tag_mesa → tag_carrito1` con valores distintos. TF2 alternaba entre ambas fuentes.
- **Solución:** Documentar como regla: "Nunca mezcles modos." Agregar advertencia prominente en README y launch file.
- **Lección:** En TF2, un frame debe tener exactamente UN publicador. No hay prioridades ni override — solo caos.
- **Archivos:** `README.md`, `education/talleres/TALLER_URDF_TF.md` (sección de errores comunes)

---

### [2026-04] — Lag de 500ms entre MoveIt y los motores del Kinova
- **Problema:** Había medio segundo de delay entre que MoveIt emitía un waypoint y el brazo empezaba a moverse.
- **Causa raíz:** El `RouterClientSendOptions` del driver tenía un timeout de 500ms-1000ms por default.
- **Solución:** Reducir a 200ms vía el script de parcheo.
- **Lección:** Los timeouts conservadores de los drivers son para demos, no para producción. Medir la latencia end-to-end antes de culpar a la red.
- **Archivos:** `scripts/apply_kinova_smooth_movement.py`, `docs/manipulation/MEJORAS_MOVIMIENTO_KINOVA.md`

---

### [2026-05] — Cámara RTSP del Kinova rechazaba conexión
- **Problema:** `test_kinova_camera.py` no podía conectar al stream RTSP.
- **Causa raíz:** Faltaban las credenciales `admin:admin` en la URL RTSP.
- **Solución:** Documentar las credenciales por defecto en `VERIFICACION_CAMARA.md`.
- **Lección:** Siempre documentar credenciales de fábrica. El error "connection refused" rara vez es de red — suele ser de autenticación.
- **Archivos:** `vision_setup/VERIFICACION_CAMARA.md`

---

### [2026-05] — Calibración incorrecta de `tag_mesa` propagaba error a todos los carritos
- **Problema:** Los carritos aparecían 8cm desplazados de su posición real.
- **Causa raíz:** El joint `table_to_tag_mesa` en el URDF tenía coordenadas estimadas "a ojo" que no coincidían con la posición real del sticker.
- **Solución:** Medir con cinta métrica la posición exacta del AprilTag respecto al origen de la mesa y actualizar el URDF.
- **Lección:** La regla "calibra de raíz a hojas" en acción. Un error de 5cm en `tag_mesa` se convierte en 5cm de error en TODOS los carritos.
- **Archivos:** `burger_description/urdf/delivery_scene_fixed.urdf` (joint `table_to_tag_mesa`)

---

*— Fin del log actual. Se agregan entradas a medida que se resuelven nuevos problemas. —*
