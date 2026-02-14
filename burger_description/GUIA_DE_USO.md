# Guía de Uso y Lanzamiento - Robot Burger

**Nota Importante:** El comando `lanzar_robot` es un **atajo personalizado** que configuramos en tu PC. Si llevas este paquete a otro computador, necesitarás configurar el atajo primero (ver sección "Configuración Inicial" abajo).

## 0. Configuración Inicial (Solo primera vez en un PC nuevo) ⚙️

Si estás en un computador nuevo, ejecuta esto una sola vez para crear los atajos:

**En PowerShell (Windows):**
```powershell
# Abre tu perfil de PowerShell
notepad $PROFILE

# Pega esto al final del archivo y guarda:
function lanzar_robot {
    wsl -d Ubuntu bash -c "source ~/ros2_ws/install/setup.bash && ros2 launch burger_description display.launch.py & sleep 5 && rqt"
}

# Recarga el perfil
. $PROFILE
```

**En WSL (Ubuntu):**
```bash
# Agrega el alias a tu .bashrc
echo "alias lanzar_robot='source ~/ros2_ws/install/setup.bash && ros2 launch burger_description display.launch.py & sleep 5 && rqt'" >> ~/.bashrc
source ~/.bashrc
```

---

## 1. Opción Rápida: Desde Windows (PowerShell) 🚀

Hemos configurado un comando directo para que no tengas que abrir WSL manualmente.

**Requisitos:**
- Tener **XLaunch (VCXSRV)** corriendo en Windows (configurado con "Disable access control").

**Pasos:**
1. Abre una terminal de **PowerShell**.
2. Escribe el comando:
   ```powershell
   lanzar_robot
   ```

**¿Qué hace este comando?**
- Conecta automáticamente con tu instancia de WSL (Ubuntu).
- Carga el entorno de ROS 2.
- Lanza la visualización del robot (RViz) con la configuración correcta.
- Abre RQT para controles adicionales.

---

## 2. Desde la Terminal de WSL (Ubuntu) 🐧

Si ya estás dentro de tu terminal de Ubuntu, también puedes usar el alias corto.

**Pasos:**
1. En tu terminal de Ubuntu, escribe:
   ```bash
   lanzar_robot
   ```

---

## 3. Lanzamiento Manual (Paso a Paso) 🛠️

Si necesitas depurar o cambiar parámetros, usa los comandos completos.

**Terminal 1: Visualización y Estado del Robot**
```bash
# 1. Cargar el entorno
source ~/ros2_ws/install/setup.bash

# 2. Lanzar el archivo principal
ros2 launch burger_description display.launch.py
```

**Terminal 2: Herramientas Gráficas (Opcional)**
```bash
source ~/ros2_ws/install/setup.bash
rqt
```

---

## 4. Solución de Problemas Comunes 🚑

### RViz no abre o da error "Could not connect to display"
- **Causa:** El servidor gráfico X (VCXSRV) no está corriendo en Windows o está mal configurado.
- **Solución:**
  1. Cierra XLaunch si está abierto.
  2. Ábrelo de nuevo.
  3. Asegúrate de marcar la casilla **"Disable access control"** en la última pantalla de configuración.

### No se ven los carritos o el brazo
- **Causa:** Problema con el "Fixed Frame" en RViz.
- **Solución:**
  1. En RViz, panel izquierdo "Global Options".
  2. Cambia **Fixed Frame** a `map`.

### Error de "Mesh not found"
- **Causa:** Rutas incorrectas en el URDF.
- **Solución:** Asegúrate de haber compilado el paquete después de cualquier cambio:
  ```bash
  cd ~/ros2_ws
  colcon build --packages-select burger_description
  source install/setup.bash
  ```
