# 🔄 Guía Oficial de Sincronización y Actualizaciones — Corte 1 (2026-2)

> **Asignatura:** ELECTIVA ELECTRONICA ( ROBOT OPERATING SYSTEM )  
> **Periodo:** 2026-2  
> **Tag oficial del corte:** `base-2026-2-corte1`  
> **Repositorio docente (`upstream`):** [`https://github.com/roncanciovl/burger_delivery`](https://github.com/roncanciovl/burger_delivery)  
> **Organización oficial:** [`https://github.com/umng-mecatronica-ros`](https://github.com/umng-mecatronica-ros)

---

## 📌 1. Propósito de este documento

Este documento detalla los cambios y adiciones incorporados en la versión base oficial del **Primer Corte (`base-2026-2-corte1`)** y establece el **procedimiento estándar y seguro** para que cada equipo sincronice estas actualizaciones en su repositorio privado asignado (`burger-kinova-equipo-XX`) sin sobreescribir su propio trabajo ni violar las políticas de control de versiones.

---

## 🚀 2. Resumen de novedades y cambios en la Base Corte 1

La etiqueta `base-2026-2-corte1` consolida la estructura curricular y técnica definitiva para el desarrollo del primer corte:

1. **🎓 Taller de Fundamentos de GitHub y Perfil Profesional:**
   * Inclusión de [`education/talleres/TALLER_GITHUB_INICIO_PERFIL_REPOSITORIOS.md`](../talleres/TALLER_GITHUB_INICIO_PERFIL_REPOSITORIOS.md) con guía de autenticación 2FA, creación del Profile README `username/username` y exploración de repositorios.
2. **🤖 Módulos de micro-ROS y Percepción 2D:**
   * Inclusión de [`education/talleres/TALLER_MICROROS_ESP32_ROBOTICA_MOVIL.md`](../talleres/TALLER_MICROROS_ESP32_ROBOTICA_MOVIL.md) para arquitecturas XRCE-DDS en ESP32.
   * Inclusión de [`education/talleres/TALLER_LOCALIZACION_APRILTAG_KINOVA_MICROROS.md`](../talleres/TALLER_LOCALIZACION_APRILTAG_KINOVA_MICROROS.md) para localización fiduciaria directa sin sobrecargar TF2.
3. **📊 Instrumentos de Evidencia y Evaluación ABET:**
   * Plantillas formales y actualizadas de recolección de evidencias para el Proyecto de Corte 1 y los laboratorios 01 y 02 en `education/evidencias_abet/`.
4. **🛠️ Modernización de comandos de Git y configuración de red:**
   * Migración de comandos legados `git checkout` a la sintaxis moderna `git switch`.
   * Unificación de rutas de remotos `upstream` con protección de escritura (`push DISABLED`).

---

## 🛠️ 3. Procedimiento paso a paso para sincronizar tu equipo

Sigue este procedimiento en la terminal de tu máquina Ubuntu (`~/ros2_ws/src/burger_delivery`):

### Paso 1: Configurar el remoto del docente (`upstream`)
Verifica si ya tienes configurado el repositorio del docente como `upstream`.

```bash
cd ~/ros2_ws/src/burger_delivery
git remote -v
```

Si no aparece `upstream`, agrégalo según el protocolo que utilices:

* **Si usas SSH (Recomendado):**
  ```bash
  git remote add upstream git@github.com:roncanciovl/burger_delivery.git
  ```
* **Si usas HTTPS:**
  ```bash
  git remote add upstream https://github.com/roncanciovl/burger_delivery.git
  ```

* **🔒 Protección obligatoria de sólo lectura:**
  ```bash
  git remote set-url --push upstream DISABLED
  ```

---

### Paso 2: Descargar las etiquetas del docente
Descarga todos los commits y tags publicados por el docente sin alterar tus ramas locales:

```bash
git fetch upstream --tags
```

---

### Paso 3: Crear una rama de sincronización
Crea una rama dedicada a la integración que parta del último estado de `main` en el repositorio de tu equipo:

```bash
# Asegurarse de tener la última versión de origin
git fetch origin
git switch -c sync/base-2026-2-corte1 origin/main
```

---

### Paso 4: Realizar la fusión explícita (`merge --no-ff`)
Fusiona el tag oficial en tu rama de sincronización:

```bash
git merge --no-ff refs/tags/base-2026-2-corte1
```

> [!NOTE]
> La bandera `--no-ff` (no fast-forward) es obligatoria porque crea un commit de merge explícito que certifica la fecha y el punto exacto en el que el equipo incorporó la base del docente.

* **Si no hay conflictos:** Git abrirá el editor para guardar el mensaje del commit de merge. Guárdalo y ciérralo.
* **Si hay conflictos:** Abre los archivos marcados, resuelve las diferencias en conjunto con tu equipo, ejecuta `git add <archivos>` y finaliza con `git commit`.

---

### Paso 5: Publicar la rama y abrir Pull Request en GitHub

1. **Subir la rama al repositorio de tu equipo:**
   ```bash
   git push -u origin sync/base-2026-2-corte1
   ```
2. **Abrir Pull Request en GitHub:**
   * Entra a `https://github.com/umng-mecatronica-ros/burger-kinova-equipo-XX`.
   * Verás el banner para crear el Pull Request desde `sync/base-2026-2-corte1` hacia `main`.
   * Titúlalo: `chore: sincronizar base oficial corte 1 (base-2026-2-corte1)`.
3. **Revisión por pares:**
   * El compañero de equipo revisa los cambios y aprueba el PR.
   * Se completa el **Merge Pull Request** hacia `main`.

---

### Paso 6: Actualizar tu clon local
Una vez hecho el merge en GitHub, cada integrante actualiza su rama `main` local:

```bash
git switch main
git pull --ff-only origin main
```

---

## ❓ 4. Preguntas Frecuentes y Buenas Prácticas

### ¿Varios computadores pueden ejecutar el driver del robot (`kortex_driver`) al mismo tiempo?
**No. Está estrictamente prohibido.** Ejecutar el driver en múltiples computadores para el mismo robot físico o en el mismo dominio ROS 2 produce colisiones críticas:
1. **Bloqueo del robot:** La controladora del Kinova Gen3 sólo admite una sesión cíclica TCP/UDP en tiempo real. Múltiples conexiones disparan paradas de seguridad (*Safety Faults*).
2. **Colisión de interfaces ROS 2:** Se duplican `/joint_states`, `/tf`, `/controller_manager` y el Action Server `/joint_trajectory_controller/follow_joint_trajectory`.

**Arquitectura correcta:**
* **Estación A (físicamente conectada al robot):** `start_driver:=true robot_ip:=192.168.1.10`
* **Estación B (estudiantes/clientes):** `start_driver:=false` (consume telemetría y envía metas a través de DDS).
* **Simulación individual (modo Fake):** Si prueban con `use_fake_hardware:=true`, cada equipo debe usar un `ROS_DOMAIN_ID` distinto para no interferir en la red.

### ¿Por qué no debemos hacer `git pull upstream main` directamente sobre `main`?
Porque hacer pull directo a `main` viola la regla de gobernanza del curso (*no push directo a main*), no deja evidencia de revisión por pares en GitHub y puede provocar mezclas accidentales de ramas difíciles de auditar en la evaluación ABET.

### ¿Qué pasa si el docente publica una corrección posterior (ej. `base-2026-2-corte1-fix1`)?
El procedimiento es exactamente el mismo: sólo cambiarás el nombre del tag en el Paso 3 y 4 (`sync/base-2026-2-corte1-fix1` y `refs/tags/base-2026-2-corte1-fix1`).

### ¿Cómo confirmamos que la versión de entrega está lista?
Al momento de la entrega final, se crea el tag de entrega inmutable:
```bash
git tag -a entrega-2026-2-corte1 -m "Entrega oficial 2026-2 - corte 1"
git push origin entrega-2026-2-corte1
git rev-parse HEAD
```
El SHA resultante es el identificador definitivo registrado en la evaluación.
