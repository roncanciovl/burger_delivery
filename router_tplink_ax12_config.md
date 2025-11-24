# Guía de Configuración para TP-Link Archer AX12 (AX1500)

## 🎯 Diagnóstico Rápido

Si **funciona con hotspot de celular** pero **NO con el router TP-Link**, el problema está en la configuración del router.

## 🔑 Acceso al Router

1. **URL de acceso:**
   - `http://192.168.1.1`
   - `http://tplinkwifi.net`

2. **Credenciales por defecto:**
   - Usuario: `admin`
   - Contraseña: (la que configuraste en el setup inicial, o `admin` si nunca la cambiaste)

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

---

## 🔍 Información que el Script Extrae Automáticamente

Al ejecutar `diagnostico_microros.ps1` (Windows) o `diagnostico_microros.sh` (Linux), obtendrás:

✅ **Gateway/Router IP:** Dirección del router  
✅ **SSID Conectado:** Verifica que sea "ros2"  
✅ **MAC Address del PC:** Necesaria para la reserva DHCP  
✅ **DNS Servers:** Servidores DNS configurados  
✅ **Rutas de Red:** Tabla de rutas completa  
✅ **Conectividad al Router:** Si responde a ping  

---

## 📋 Checklist de Configuración

Usa este checklist para verificar el router:

```
[ ] 1. Accedí al router en http://192.168.1.1
[ ] 2. AP Isolation está DESACTIVADO
[ ] 3. Smart Connect está desactivado (para pruebas)
[ ] 4. SSID es "ros2" con password "ros12345"
[ ] 5. DHCP activo en rango 192.168.1.101-254
[ ] 6. IP 192.168.1.100 reservada para mi PC (por MAC)
[ ] 7. Firewall no está bloqueando tráfico interno
[ ] 8. No hay reglas de Access Control activas
[ ] 9. Reinicié el router después de los cambios
[ ] 10. Reconecté el PC y las ESP32s a la red
```

---

## 🚀 Prueba de Validación

Después de configurar el router:

1. **Ejecutar el agente micro-ROS:**
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
   ```

2. **Ejecutar el script de diagnóstico:**
   ```powershell
   .\diagnostico_microros.ps1
   ```

3. **Reiniciar las ESP32s** y verificar en los logs del agente que se conectan.

4. **Verificar tópicos ROS:**
   ```bash
   ros2 topic list
   ```
   Deberías ver los tópicos de los robots, por ejemplo:
   ```
   /robot_A/odom
   /robot_A/cmd_vel
   /robot_B/odom
   /robot_B/cmd_vel
   ```

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
