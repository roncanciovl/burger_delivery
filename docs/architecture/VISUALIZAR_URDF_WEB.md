# Visualizar el URDF en el visor web (URDF-Loaders)

Este documento explica cómo abrir y revisar tus modelos URDF en el visor online basado en three.js, útil si no tienes acceso inmediato a RViz o a un entorno ROS 2 completo.

Visor web: [https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html](https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/index.html)

## Requisitos
- La carpeta principal de tu paquete ROS 2: `burger_description`
- En la nueva estructura, los modelos URDF con geometrías (visuales y colisiones) y mallas se encuentran dentro de la carpeta `burger_description/`.
  - URDFs: `burger_description/urdf/`
  - Mallas visuales: `burger_description/visual/`
  - Mallas de vendor (Kinova/Robotiq): `burger_description/vendor/`

## Pasos rápidos (local)
1. Abre el visor web en tu navegador.
2. Dado que los archivos URDF utilizan etiquetas de ROS como `package://burger_description/...` para encontrar las mallas, **debes arrastrar y soltar la carpeta `burger_description` COMPLETA** dentro de la ventana del visor.
3. El visor reconocerá que es un paquete y resolverá todas las rutas relativas.
4. En el panel lateral izquierdo del visor, si no se carga de inmediato, se te pedirá seleccionar un archivo principal. Selecciona:
   - `urdf/delivery_scene_fixed.urdf` (escena principal actual).
   - o `urdf/burger_delivery_gen3.urdf` (variante alternativa).
5. En el panel derecho de opciones, activa **"Show Axes"** y **"Show Link Names"** para identificar claramente los marcos de referencia (frames) de los robots, las cámaras y las mesas.
6. Ajusta la escala si fuese necesario (debería ser 1.0; unidades en metros).

## Mapeo de `package://`
El visor necesita saber dónde encontrar el prefijo `package://burger_description`. Al arrastrar la carpeta que se llama exactamente igual que el paquete (`burger_description`), el visor asocia y mapea las rutas automáticamente de forma local en tu navegador.

## Problemas comunes y soluciones
- **No veo las geometrías (solo el esqueleto de ejes):** Probablemente no arrastraste la carpeta completa o el visor no pudo resolver las rutas `package://`. Asegúrate de arrastrar todo el directorio `burger_description` y no solo el archivo `.urdf`.
- **Error CORS al intentar cargar por URL:** Si pegas una URL directa de GitHub al URDF, puede fallar por políticas de origen cruzado o porque el visor no puede buscar de forma recursiva los archivos `.stl`/.`dae` asociados en el repositorio. Lo más fiable siempre es descargar el código y arrastrar la carpeta localmente.
- **La escala es incorrecta:** Revisa la configuración del visor. Todo en este proyecto está modelado en metros.
- **Los dedos de la pinza (gripper) se mueven por separado:** El visor web no soporta nativamente la etiqueta `<mimic>` de ROS. Por ello, te mostrará un control deslizante (slider) independiente para cada articulación y falange de la pinza, permitiéndote moverlas por separado en la web. No te preocupes, esto es solo un límite técnico del visor online; en tu robot físico y en ROS 2 (RViz), la pinza funciona de forma coordinada con un solo comando.

## Controles de navegación en el visor
- **Rotar cámara:** clic izquierdo + arrastrar.
- **Desplazar cámara (pan):** clic derecho o Shift + arrastrar.
- **Zoom:** rueda del ratón o gesto de pellizco en el trackpad.
- **Reset:** pulsa el botón para centrar la cámara en el origen.
