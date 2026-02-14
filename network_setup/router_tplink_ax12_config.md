# Guía de Configuración para TP-Link Archer AX12 (AX1500)


## 🔑 Acceso al Router

1. **URL de acceso:**
   - `http://192.168.1.1`
   - `http://tplinkwifi.net`

2. **Credenciales por defecto:**
   - Usuario: `admin`
   - Contraseña: (ros123, o `admin` si nunca la cambiaste)

---

## ⚙️ Configuraciones Críticas a Verificar

### 1. **AP Isolation (CAUSA MÁS COMÚN)** ⚠️

**Problema:** Si está activado, los dispositivos WiFi no pueden comunicarse entre sí, aunque pueden hacer ping.

**Ruta de acceso:**
```
Advanced → Wireless → Wireless Settings
```

**Qué buscar:**
- Sección: **"Guest Network"** o **"Main Network Settings"**
- Opción: **"Enable AP Isolation"** o **"Isolate Clients"**
- **DEBE ESTAR DESMARCADO (OFF)**

**Captura ejemplo:**
```
[ ] Enable AP Isolation    ← Debe estar sin marcar
```

---

### 2. **Smart Connect (WiFi 6 Feature)**

**Problema:** Puede causar desconexiones o interferencia en redes con múltiples dispositivos.

**Ruta de acceso:**
```
Advanced → Wireless → Wireless Settings
```

**Qué buscar:**
- Opción: **"Smart Connect"**
- **Recomendación:** Desactivar temporalmente para pruebas

**Notas:**
- Smart Connect combina las bandas 2.4GHz y 5GHz en un solo SSID
- Algunos dispositivos tienen problemas con esta función

---

### 3. **Configuración de Red WiFi**

**Ruta de acceso:**
```
Advanced → Wireless → Wireless Settings
```

**Configuración recomendada:**

#### Para 2.4GHz:
- **SSID:** `ros2`
- **Password:** `ros12345`
- **Security:** WPA2-PSK (o WPA2/WPA3-PSK)
- **Channel:** Auto (o manual: 1, 6, u 11)
- **Channel Width:** 20MHz o Auto
- **Mode:** 802.11b/g/n/ax mixed

#### Para 5GHz (si la usas):
- Mismas configuraciones que 2.4GHz
- O desactívala si solo usas 2.4GHz

---

### 4. **Reserva de IP (DHCP)**

**Ruta de acceso:**
```
Advanced → Network → DHCP Server
```

**Configurar:**

1. **DHCP Server:** Enabled

2. **IP Address Pool:**
   - Start IP: `192.168.1.101`
   - End IP: `192.168.1.254`

3. **Address Reservation:**
   - Clic en **"Add"** o **"Reserve"**
   - **MAC Address:** [Tu MAC del PC - el script de diagnóstico te la muestra]
   - **Reserved IP:** `192.168.1.100`
   - **Comment:** `PC Principal - micro-ROS Agent`
   - **Status:** Enabled

---

### 5. **Firewall y Seguridad**

**Ruta de acceso:**
```
Advanced → Security → Settings
```

**Para debugging inicial, puedes:**
- **SPI Firewall:** Enabled (normal)
- **DoS Protection:** Enabled (normal)

Si el problema persiste, temporalmente desactiva:
- SPI Firewall → Disabled (solo para pruebas)

**IMPORTANTE:** Vuelve a activarlo después de resolver el problema.

---

### 6. **Access Control (Control de Acceso)**

**Ruta de acceso:**
```
Advanced → Security → Access Control
```

**Verificar:**
- **Access Control:** Disabled (o si está Enabled, asegúrate de que tus dispositivos estén en la whitelist)

### 7. **Configuración de WSL (Windows Subsystem for Linux)** 💻

**Problema:** Por defecto, WSL2 usa NAT, lo que significa que tiene una IP distinta a la de Windows (ej. `172.x.x.x`) y los robots/router no pueden verlo directamente.

#### Opción A: Modo Espejo (Mirrored Mode) - RECOMENDADO ⭐
Hace que WSL comparta la misma IP que Windows (`192.168.1.100`), eliminando todos los problemas de visibilidad de red.

1. En Windows, presiona `Win + R`, escribe `%USERPROFILE%` y presiona Enter.
2. Crea o edita el archivo llamado `.wslconfig`.
3. Pega el siguiente contenido:
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
4. Apaga WSL desde una terminal de PowerShell:
   ```powershell
   wsl --shutdown
   ```
5. Al volver a abrir tu terminal de Ubuntu/Jazzy, verás que `ip addr` muestra la IP `192.168.1.100`.

#### Opción B: Port Proxy (Si no usas Modo Espejo)
Si prefieres mantener NAT, debes "puentear" el puerto UDP de micro-ROS desde Windows hacia WSL.

Ejecuta en PowerShell como **Administrador**:
```powershell
# Reemplaza 172.x.x.x con la IP que sale en 'ip addr' dentro de WSL
netsh interface portproxy add v4tov4 listenaddress=192.168.1.100 listenport=8888 connectaddress=172.x.x.x connectport=8888
```

---

## 📋 Checklist General de Configuración

Usa este checklist antes de pasar a los tests:

```
[ ] 1. Accedí al router en http://192.168.1.1
[ ] 2. AP Isolation está DESACTIVADO
[ ] 3. Smart Connect desactivado (recomendado)
[ ] 4. SSID "ros2" con password "ros12345" configurado
[ ] 5. DHCP activo y reserva de IP 192.168.1.100 para el PC
[ ] 6. Firewall/Access Control verificado
[ ] 7. Router reiniciado después de cambios
```

---

---

## 🧪 Test Básico de Conectividad (Red Local y WAN)

Antes de probar micro-ROS, asegúrate de que la red básica funciona correctamente.

### 1. Ubicación en la Subred
- Abre una terminal y verifica tu IP:
  ```powershell
  ipconfig  # Windows
  ip addr   # Linux
  ```
- **Resultado esperado:** Tu IP debe estar en el rango `192.168.1.x`.

### 2. Prueba de Salida a Internet (WAN)
- Verifica que el router tiene acceso a internet:
  ```powershell
  # Windows
  ping 8.8.8.8
  ping google.com
  ```
  ```bash
  # Linux / WSL
  ping -c 4 8.8.8.8
  ```
- **Si falla:** El cable del proveedor (ONT/Módem) debe estar en el puerto **WAN (Azul)** del TP-Link.

### 3. Prueba de Comunicación Interna (Subnet)
- Intenta hacer ping al router:
  ```powershell
  ping 192.168.1.1
  ```
- **Prueba de Fuego (AP Isolation):** Intenta hacer ping desde tu PC a la IP de una ESP32 conectada (o a otro celular/PC en la misma red).
  ```powershell
  ping 192.168.1.x  # IP de otro dispositivo
  ```
  *Si el ping al router funciona pero el ping entre dispositivos falla, el **AP Isolation** sigue activo.*

---

## 🤖 Validaciones de micro-ROS

Una vez confirmada la red, procede con las pruebas específicas de ROS2.

### 1. Iniciar el Agente
Ejecuta el agente en tu PC (asegúrate de que la IP de tu PC sea la `192.168.1.100` reservada):
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

### 2. Ejecutar Script de Diagnóstico
Usa la herramienta automatizada para verificar puertos y visibilidad:
```powershell
.\diagnostico_microros.ps1
```

### 3. Sincronización de Clientes
- Reinicia tus ESP32s físicamente o vía serial.
- Observa los logs del agente. Deberías ver:
  ```text
  [168...] Session established
  [168...] Topic matched
  ```

### 4. Verificación de Tópicos
En una nueva terminal, lista los tópicos detectados:
```bash
ros2 topic list
```
**Tópicos esperados:**
- `/robot_A/odom`
- `/robot_A/cmd_vel`
- `/robot_B/odom`
- `/robot_B/cmd_vel`

---

## 🎯 Diagnóstico Rápido (Si el script anterior falla)

Esta sección es útil si el script `diagnostico_microros.ps1` muestra errores o si el sistema **funciona con hotspot de celular** pero **NO con el router TP-Link**. En ese caso, el problema es casi seguro la configuración del router.

---

---

## 🆘 Troubleshooting Adicional

### Si aún no funciona después de los cambios:

1. **Reinicia el router completamente:**
   - Desconecta alimentación por 30 segundos
   - Vuelve a conectar

2. **Actualiza el firmware del router:**
   ```
   Advanced → System Tools → Firmware Upgrade
   ```

3. **Factory Reset (último recurso):**
   ```
   Advanced → System Tools → Backup & Restore → Factory Restore
   ```
   ⚠️ Esto borrará TODA la configuración. Anota tus settings primero.

4. **Contacto con soporte:**
   - TP-Link Support: https://www.tp-link.com/support/
   - Modelo: Archer AX12 (AX1500)

---

## 📚 Referencias

- [Manual oficial TP-Link Archer AX12](https://www.tp-link.com/en/support/download/archer-ax12/)
- [FAQ AP Isolation](https://www.tp-link.com/en/support/faq/600/)
- Documentación del proyecto: `ROS/ros.md`
