# Visualizar el URDF en el visor web (URDF-Loaders)

Este documento explica cómo abrir y revisar tu URDF en el visor online basado en three.js:

Visor: https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html

## Requisitos
- Archivos del robot: `ROS/burger_delivery_frames.urdf` (sin geometrías; solo frames y joints).
- Si tu URDF hace referencia a `package://` o mallas (`.stl`, `.dae`, `.obj`), ten esos archivos a mano en carpetas locales para poder mapear rutas. En este proyecto, el archivo define solo frames (no mallas).

## Pasos rápidos (local)
1) Abre el visor web en tu navegador.
2) Opción A (frames): arrastra y suelta `ROS/burger_delivery_frames.urdf`.
3) Opción B (con geometrías): arrastra y suelta `ROS/visual/burger_delivery_visual.urdf` y asegúrate de que el visor también tenga acceso a `ROS/visual/meshes/cube_1m.stl` (si arrastras la carpeta `visual/` completa es más fácil).
4) En el panel de opciones, activa ejes/nombres ("Show Axes", "Show Names/Labels") para ver claramente los frames. En la versión visual ya verás cubos coloreados.
5) Ajusta la escala si fuese necesario (debería ser 1.0; unidades en metros).
6) Usa la cuadrícula y la iluminación para orientar la vista (Grid/Floor/Environment).

## Cargar por URL (opcional)
Si tu URDF está publicado online (por ejemplo, en GitHub Raw), puedes pegar la URL directa del `.urdf` en el cuadro de "URL" del visor y presionar "Load". Nota: si el URDF referencia mallas con `package://`, también necesitarás configurar el mapeo de recursos.

## Mapeo de `package://` (si aplica)
En el panel del visor suele haber un apartado de "Resource Map" o similar. Agrega entradas del estilo:
- Prefijo: `package://tu_paquete/`
- Ruta local: carpeta donde estén las mallas de ese paquete.

Para `burger_delivery_frames.urdf` no es necesario, porque no usa mallas. Para `burger_delivery_visual.urdf` usamos rutas relativas (por ejemplo `meshes/cube_1m.stl`) que el visor resuelve si cargas también ese archivo o arrastras la carpeta.

## Controles de navegación (estándar)
- Rotar cámara: clic izquierdo + arrastrar.
- Desplazar (pan): clic derecho o Shift + arrastrar.
- Zoom: rueda del ratón o gesto de pellizco.
- Reset de vista: botón "Reset"/"Home" si está disponible.

## Qué verificar en este proyecto
Este URDF modela un conjunto de frames de referencia (sin visuales):
- `map`: frame raíz.
- `kinova_base_link`: ahora está a 1.00 m sobre el piso (z = 1.00) desde `map`.
- `kinova_tool_frame` y `burger_grip_frame`: hijos de la base del Kinova (el grip está a +0.18 m del tool frame).
- `robot_a_base_link` y `robot_b_base_link`: robots auxiliares con bandejas a +0.15 m.
- `overhead_camera_link`: cámara aérea en `z = 2.50` con orientación fija.
- `staging_area` y `delivery_slot_1`: zona de staging y slot a `z = 0.90` relativo a `staging_area`.

Sugerencias:
- Activa ejes/nombres para confirmar posiciones relativas entre frames.
- Mide distancias visualmente con la cuadrícula (1 unidad = 1 m).

## Variante con visuales
- Ruta: `ROS/visual/burger_delivery_visual.urdf`
- Incluye una mesa (`table_link`) como caja delgada (0.05 m) cuyo tope queda a 1.0 m, la base del Kinova a z = 1.0, y cubos de colores para los demás links (trays, robots A/B, cámara, staging/slot).
- Las mallas se escalan a partir de `ROS/visual/meshes/cube_1m.stl`.

Nombres y frames:
- Cada `visual` tiene un nombre descriptivo (por ejemplo `table_top`, `robot_a_tray`, `overhead_camera`). Algunos visores muestran estos nombres en sus árboles/propiedades.
- Se añadieron marcadores de ejes por link (barras): X rojo, Y verde, Z azul. Así puedes ver el frame de cada link incluso si el visor no muestra ejes automáticamente.

En el visor, también puedes activar:
- "Show Link Names" para ver los nombres de `link`.
- "Show Axes" para mostrar ejes del visor (complementa los marcadores incluidos).

Flechas con cabeza:
- Los ejes por link ahora incluyen puntas (pirámides) para indicar el sentido positivo: X→ rojo, Y→ verde, Z→ azul. Si no aparecen, asegúrate de haber cargado también la carpeta `ROS/visual/meshes/` junto con el URDF.

Consejo: arrastra la carpeta `ROS/visual/` completa al visor para que resuelva automáticamente las rutas relativas de malla.

## Problemas comunes y soluciones
- No veo el robot: este archivo no define geometrías; debes activar ejes/labels para ver los frames.
- Mallas que no cargan: necesitas mapear `package://` a carpetas locales en "Resource Map" y/o arrastrar también los `.stl/.dae` referenciados.
- CORS al cargar por URL: usa GitHub Raw u otro hosting que permita acceso directo, o carga los archivos de forma local (drag & drop).
- Escala rara: revisa la opción de escala/unidades del visor (debe ser metros).

## Archivo a usar
- Ruta: `ROS/burger_delivery_frames.urdf`

## Modelo Kinova Gen3 con gripper (oficial)
- Rutas vendorizadas:
  - URDF listo para visor: `ROS/vendor/kortex_description/robots/gen3_2f85.urdf`
  - Mallas del brazo: `ROS/vendor/kortex_description/arms/gen3/7dof/meshes/`
  - Mallas del gripper (Robotiq 2F-85): `ROS/vendor/robotiq_description/meshes/{visual,collision}/2f_85/`

Cómo verlo solo:
- Arrastra `ROS/vendor/kortex_description/robots/gen3_2f85.urdf` y la carpeta `ROS/vendor/` al visor.
- He ajustado las rutas de malla a relativas, y sustituido la malla de la cámara de muñeca por una caja para evitar dependencias externas (Realsense).

## Escena compuesta (burger + Gen3)
- Ruta: `ROS/visual/burger_delivery_gen3.urdf`
- Contiene la escena de `burger_delivery_visual` y el robot Gen3 (2F-85) en la posición: `map -> world` a `xyz="1.20 0.30 1.00"` y de ahí `world -> gen3_base_link`.
 - Las bases de `robot_a_base_link` y `robot_b_base_link` son joints `planar` (movimiento en XY y yaw sobre Z). Posición inicial al mismo lado (Y positivo) de la base del Kinova: `robot_a` en `xyz="1.20 0.80 0.00"`, `robot_b` en `xyz="1.20 1.30 0.00"`.

Para cargarla en el visor:
- Ahora basta con arrastrar la carpeta `ROS/visual/` completa. Luego abre `burger_delivery_gen3.urdf` desde el panel si no se carga automáticamente.
- Activa "Show Link Names" para distinguir `map`, `world`, `gen3_*` y los demás links.

Si no ves la geometría del Kinova:
- Verifica que dentro del visor aparezcan subcarpetas `visual/meshes/` y `visual/vendor/` (con `kortex_description` y `robotiq_description`).
- Asegúrate de haber arrastrado la carpeta `ROS/visual/` completa (no solo el URDF).
