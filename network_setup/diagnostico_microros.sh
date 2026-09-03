#!/bin/bash
# diagnostico_microros.sh
# Script de diagnóstico para depurar problemas de comunicación micro-ROS
# Uso: ./diagnostico_microros.sh

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color

echo -e "${CYAN}=====================================${NC}"
echo -e "${CYAN} Diagnóstico de micro-ROS Network${NC}"
echo -e "${CYAN}=====================================${NC}"

# Configuración. Se autodetecta para funcionar en cualquier máquina; puedes
# sobrescribirla desde el entorno sin editar el script:
#   AGENT_IP=192.168.1.100 ESP32_IPS="192.168.1.101 192.168.1.102" bash diagnostico_microros.sh
detectar_ip_local() {
    # IP de origen de la ruta por defecto: es la que ven los demás equipos.
    local ip
    ip=$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i=="src") {print $(i+1); exit}}')
    if [ -z "$ip" ]; then
        # Respaldo: primera IP global que no esté en la interfaz loopback.
        ip=$(ip -4 -brief addr show scope global 2>/dev/null | awk '$1!="lo" {split($3, a, "/"); print a[1]; exit}')
    fi
    echo "$ip"
}

IP_LOCAL=$(detectar_ip_local)
AGENT_IP="${AGENT_IP:-$IP_LOCAL}"
AGENT_PORT="${AGENT_PORT:-8888}"
read -r -a ESP32_IPS <<< "${ESP32_IPS:-192.168.1.101 192.168.1.102}"
ROS_DOMAIN_ID_ESPERADO="${ROS_DOMAIN_ID_ESPERADO:-0}"

echo -e "\n${YELLOW}[INFO] Configuración actual:${NC}"
echo "  - Agent IP: $AGENT_IP (IP local detectada; los ESP32 deben apuntar aquí)"
echo "  - Agent Port: $AGENT_PORT"
echo "  - ESP32s a verificar: ${ESP32_IPS[*]}"

# ======================================
# 1. Verificar IP del PC Principal
# ======================================
echo -e "\n${YELLOW}[1/9] Verificando IP del PC Principal...${NC}"
CURRENT_IP="$IP_LOCAL"

if [ -z "$CURRENT_IP" ]; then
    echo -e "  ${RED}❌ No se pudo determinar la IP${NC}"
elif [ "$CURRENT_IP" == "$AGENT_IP" ]; then
    echo -e "  ${GREEN}✅ IP del agente: $CURRENT_IP${NC}"
    echo -e "     ${NC}Este valor debe coincidir con 'agent_ip' en el firmware de los ESP32.${NC}"
else
    echo -e "  ${YELLOW}⚠️  IP actual: $CURRENT_IP${NC}"
    echo -e "     Esperada: $AGENT_IP"
    echo -e "  ${RED}Verifica la reserva DHCP en el router o corrige AGENT_IP${NC}"
fi

# ======================================
# 2. Verificar estado del Firewall
# ======================================
echo -e "\n${YELLOW}[2/9] Verificando estado del Firewall...${NC}"

# Verificar si ufw está instalado
if command -v ufw &> /dev/null; then
    UFW_STATUS=$(sudo ufw status 2>/dev/null | grep -i "Status:" | awk '{print $2}')
    if [ "$UFW_STATUS" == "active" ]; then
        echo -e "  ${YELLOW}⚠️  UFW: ACTIVO (puede bloquear micro-ROS)${NC}"
        
        # Verificar si existe regla para el puerto
        UFW_RULE=$(sudo ufw status | grep $AGENT_PORT 2>/dev/null)
        if [ -n "$UFW_RULE" ]; then
            echo -e "  ${GREEN}✅ Regla UFW para puerto $AGENT_PORT encontrada${NC}"
        else
            echo -e "  ${YELLOW}⚠️  No hay regla UFW para puerto $AGENT_PORT${NC}"
            echo -e "  ${CYAN}Ejecuta: sudo ufw allow $AGENT_PORT/udp${NC}"
        fi
    else
        echo -e "  ${GREEN}✅ UFW: Desactivado${NC}"
    fi
else
    echo -e "  ${WHITE}ℹ️  UFW no está instalado${NC}"
fi

# Verificar iptables
if command -v iptables &> /dev/null; then
    IPTABLES_RULES=$(sudo iptables -L -n | grep -c "$AGENT_PORT")
    if [ "$IPTABLES_RULES" -gt 0 ]; then
        echo -e "  ${WHITE}ℹ️  Reglas iptables activas para puerto $AGENT_PORT${NC}"
    fi
fi

# ======================================
# 3. Verificar si el puerto 8888 está en uso
# ======================================
echo -e "\n${YELLOW}[3/9] Verificando puerto UDP $AGENT_PORT...${NC}"

if command -v ss &> /dev/null; then
    PORT_INFO=$(ss -ulnp 2>/dev/null | grep ":$AGENT_PORT")
    if [ -n "$PORT_INFO" ]; then
        echo -e "  ${GREEN}✅ Puerto $AGENT_PORT UDP está en uso:${NC}"
        echo "$PORT_INFO" | while read line; do
            echo -e "     ${WHITE}$line${NC}"
        done
    else
        echo -e "  ${RED}❌ Puerto $AGENT_PORT UDP NO está en uso${NC}"
        echo -e "  ${RED}El agente micro-ROS no está ejecutándose${NC}"
        echo -e "  ${CYAN}Ejecuta: ros2 run micro_ros_agent micro_ros_agent udp4 --port $AGENT_PORT -v6${NC}"
    fi
elif command -v netstat &> /dev/null; then
    PORT_INFO=$(netstat -ulnp 2>/dev/null | grep ":$AGENT_PORT")
    if [ -n "$PORT_INFO" ]; then
        echo -e "  ${GREEN}✅ Puerto $AGENT_PORT UDP está en uso${NC}"
    else
        echo -e "  ${RED}❌ Puerto $AGENT_PORT UDP NO está en uso${NC}"
    fi
else
    echo -e "  ${YELLOW}⚠️  No se pudo verificar (ss/netstat no disponibles)${NC}"
fi

# ======================================
# 4. Ping a ESP32s
# ======================================
echo -e "\n${YELLOW}[4/9] Verificando conectividad ICMP (ping) a ESP32s...${NC}"
for esp_ip in "${ESP32_IPS[@]}"; do
    if ping -c 2 -W 1 "$esp_ip" &>/dev/null; then
        echo -e "  ${GREEN}✅ $esp_ip responde a ping${NC}"
    else
        echo -e "  ${RED}❌ $esp_ip NO responde a ping${NC}"
        echo -e "  ${RED}Verifica que la ESP32 esté conectada a la red WiFi 'ros2'${NC}"
    fi
done
# Verificar que la IP de cada ESP32 está en la misma subred que el agente
AGENT_SUBNET=$(echo "$AGENT_IP" | cut -d'.' -f1-3)
for esp_ip in "${ESP32_IPS[@]}"; do
    ESP_SUBNET=$(echo "$esp_ip" | cut -d'.' -f1-3)
    if [ "$ESP_SUBNET" = "$AGENT_SUBNET" ]; then
        echo -e "  ${GREEN}✅ $esp_ip está en la subred $AGENT_SUBNET.x${NC}"
    else
        echo -e "  ${YELLOW}⚠️  $esp_ip está en una subred diferente ($ESP_SUBNET.x)${NC}"
    fi
done
# ======================================
# 5. Verificar variables de entorno ROS
# ======================================
echo -e "\n${YELLOW}[5/9] Verificando variables de entorno ROS 2...${NC}"

if [ -n "$ROS_DOMAIN_ID" ]; then
    if [ "$ROS_DOMAIN_ID" == "$ROS_DOMAIN_ID_ESPERADO" ]; then
        echo -e "  ${GREEN}✅ ROS_DOMAIN_ID = $ROS_DOMAIN_ID (coincide con el dominio del proyecto)${NC}"
    else
        echo -e "  ${YELLOW}⚠️  ROS_DOMAIN_ID = $ROS_DOMAIN_ID (el proyecto usa $ROS_DOMAIN_ID_ESPERADO)${NC}"
        echo -e "     ${NC}No es un error si tu grupo tiene un dominio asignado, pero TODAS${NC}"
        echo -e "     ${NC}las máquinas y el agente micro-ROS deben usar el mismo valor.${NC}"
    fi
else
    echo -e "  ${GREEN}✅ ROS_DOMAIN_ID no configurado (usa default 0)${NC}"
fi

if [ -n "$ROS_LOCALHOST_ONLY" ]; then
    if [ "$ROS_LOCALHOST_ONLY" == "0" ] || [ "$ROS_LOCALHOST_ONLY" == "false" ]; then
        echo -e "  ${GREEN}✅ ROS_LOCALHOST_ONLY = $ROS_LOCALHOST_ONLY (permite red)${NC}"
    else
        echo -e "  ${RED}❌ ROS_LOCALHOST_ONLY = $ROS_LOCALHOST_ONLY (BLOQUEA comunicación de red!)${NC}"
        echo -e "  ${CYAN}Configura: export ROS_LOCALHOST_ONLY=0${NC}"
    fi
else
    echo -e "  ${GREEN}✅ ROS_LOCALHOST_ONLY no configurado (permite red)${NC}"
fi

# ======================================
# 6. Verificar Router/Gateway
# ======================================
echo -e "\n${YELLOW}[6/9] Verificando conectividad al Router/Gateway...${NC}"

# Obtener el gateway predeterminado
GATEWAY=$(ip route | grep default | awk '{print $3}' | head -n 1)

if [ -n "$GATEWAY" ]; then
    echo -e "  ${WHITE}Gateway detectado: $GATEWAY${NC}"
    
    # Verificar conectividad al router
    if ping -c 2 -W 1 "$GATEWAY" &>/dev/null; then
        echo -e "  ${GREEN}✅ Router responde a ping${NC}"
        
        # Verificar que el gateway esté en la misma subred que el agente
        AGENT_SUBNET=$(echo "$AGENT_IP" | cut -d'.' -f1-3)
        GATEWAY_SUBNET=$(echo "$GATEWAY" | cut -d'.' -f1-3)
        
        if [ "$AGENT_SUBNET" == "$GATEWAY_SUBNET" ]; then
            echo -e "  ${GREEN}✅ Router y agente en la misma subred ($AGENT_SUBNET.x)${NC}"
        else
            echo -e "  ${YELLOW}⚠️  Router ($GATEWAY_SUBNET.x) y agente ($AGENT_SUBNET.x) en subredes diferentes${NC}"
        fi
    else
        echo -e "  ${RED}❌ Router NO responde a ping${NC}"
        echo -e "  ${RED}Problema de conectividad con el router. Verifica:${NC}"
        echo -e "  ${YELLOW}- Conexión WiFi o cable de red${NC}"
        echo -e "  ${YELLOW}- Configuración del adaptador de red${NC}"
    fi
    
    echo -e "\n  ${CYAN}📋 Información de Red Detallada:${NC}"
    
    # Información del adaptador WiFi
    WIFI_INTERFACE=$(ip link show | grep -i "wlan\|wifi" | head -n 1 | awk -F: '{print $2}' | xargs)
    if [ -n "$WIFI_INTERFACE" ]; then
        echo -e "  ${WHITE}Adaptador WiFi: $WIFI_INTERFACE${NC}"
        MAC_ADDRESS=$(ip link show "$WIFI_INTERFACE" | grep ether | awk '{print $2}')
        echo -e "  ${WHITE}MAC Address: $MAC_ADDRESS${NC}"
        
        # Intentar obtener SSID conectado
        if command -v iwgetid &> /dev/null; then
            SSID=$(iwgetid -r 2>/dev/null)
            if [ -n "$SSID" ]; then
                echo -e "  ${WHITE}SSID Conectado: $SSID${NC}"
                if [ "$SSID" != "ros2" ]; then
                    echo -e "  ${YELLOW}⚠️  ADVERTENCIA: No estás conectado al SSID 'ros2'${NC}"
                fi
            fi
        elif command -v nmcli &> /dev/null; then
            SSID=$(nmcli -t -f active,ssid dev wifi | grep '^yes' | cut -d':' -f2)
            if [ -n "$SSID" ]; then
                echo -e "  ${WHITE}SSID Conectado: $SSID${NC}"
                if [ "$SSID" != "ros2" ]; then
                    echo -e "  ${YELLOW}⚠️  ADVERTENCIA: No estás conectado al SSID 'ros2'${NC}"
                fi
            fi
        fi
    fi
    
    # Información DHCP y DNS
    echo -e "\n  ${CYAN}📋 Configuración DHCP y DNS:${NC}"
    
    # DNS Servers
    if [ -f /etc/resolv.conf ]; then
        DNS_SERVERS=$(grep nameserver /etc/resolv.conf | awk '{print $2}' | tr '\n' ', ' | sed 's/,$//')
        echo -e "  ${WHITE}DNS Servers: $DNS_SERVERS${NC}"
    fi
    
    # Verificar si es DHCP
    if command -v nmcli &> /dev/null; then
        DHCP_STATUS=$(nmcli -f GENERAL.STATE,IP4.ADDRESS connection show --active 2>/dev/null | grep -i dhcp)
        if [ -n "$DHCP_STATUS" ]; then
            echo -e "  ${WHITE}DHCP: Enabled${NC}"
        fi
    fi
    
    # Tabla de rutas relevante
    echo -e "\n  ${CYAN}📋 Rutas de Red:${NC}"
    ip route | grep -E "default|192.168" | head -n 5 | while read line; do
        echo -e "  ${WHITE}$line${NC}"
    done
    
    echo -e "\n  ${CYAN}📋 Configuración crítica del router a verificar manualmente:${NC}"
    echo -e "  ${WHITE}Accede a: http://$GATEWAY (o http://tplinkwifi.net para TP-Link)${NC}"
    echo -e "  "
    echo -e "  ${YELLOW}1. AP Isolation DEBE estar DESACTIVADO${NC}"
    echo -e "     ${NC}Ruta: Advanced → Wireless → Wireless Settings${NC}"
    echo -e "     ${NC}Busca: 'Enable AP Isolation' → Debe estar DESMARCADO${NC}"
    echo -e "  "
    echo -e "  ${YELLOW}2. Smart Connect (WiFi 6) - Desactivar si causa problemas${NC}"
    echo -e "     ${NC}Ruta: Advanced → Wireless → Wireless Settings${NC}"
    echo -e "  "
    echo -e "  ${YELLOW}3. IP fija reservada para este PC ($AGENT_IP)${NC}"
    if [ -n "$MAC_ADDRESS" ]; then
        echo -e "     ${NC}MAC: $MAC_ADDRESS${NC}"
    fi
    echo -e "     ${NC}Ruta: Advanced → Network → DHCP Server → Address Reservation${NC}"
    echo -e "  "
    echo -e "  ${YELLOW}4. DHCP activo en rango 192.168.1.101-254${NC}"
    echo -e "     ${NC}Ruta: Advanced → Network → DHCP Server${NC}"
    
else
    echo -e "  ${RED}❌ No se pudo detectar el gateway predeterminado${NC}"
    echo -e "  ${RED}Verifica la configuración de red${NC}"
fi

# ======================================
# 7. Verificar tabla ARP
# ======================================
echo -e "\n${YELLOW}[7/9] Verificando tabla ARP (dispositivos conocidos)...${NC}"
for esp_ip in "${ESP32_IPS[@]}"; do
    ARP_ENTRY=$(arp -n "$esp_ip" 2>/dev/null | grep -v "incomplete")
    if echo "$ARP_ENTRY" | grep -q "$esp_ip"; then
        MAC=$(echo "$ARP_ENTRY" | awk '{print $3}')
        echo -e "  ${GREEN}✅ $esp_ip está en tabla ARP (MAC: $MAC)${NC}"
    else
        echo -e "  ${YELLOW}⚠️  $esp_ip NO está en tabla ARP (sin comunicación reciente)${NC}"
    fi
done

# ======================================
# 8. Verificar conectividad UDP
# ======================================
echo -e "\n${YELLOW}[8/9] Verificando conectividad UDP...${NC}"
if command -v nc &> /dev/null; then
    echo "PING" | nc -u -w1 127.0.0.1 $AGENT_PORT &>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "  ${GREEN}✅ Envío UDP local exitoso${NC}"
    else
        echo -e "  ${YELLOW}⚠️  No se pudo enviar paquete UDP${NC}"
    fi
else
    echo -e "  ${WHITE}ℹ️  netcat (nc) no disponible, omitiendo test UDP${NC}"
fi

# ======================================
# 9. Resumen y recomendaciones
# ======================================
echo -e "\n${YELLOW}[9/9] Resumen y recomendaciones:${NC}"
echo ""

ISSUES=()

if [ "$CURRENT_IP" != "$AGENT_IP" ] && [ -n "$CURRENT_IP" ]; then
    ISSUES+=("La IP del PC no coincide con la esperada ($AGENT_IP)")
fi

if [ "$UFW_STATUS" == "active" ] && ! sudo ufw status | grep -q $AGENT_PORT; then
    ISSUES+=("Firewall UFW activo sin regla para puerto $AGENT_PORT")
fi

if ! ss -ulnp 2>/dev/null | grep -q ":$AGENT_PORT"; then
    ISSUES+=("El agente micro-ROS no está escuchando en el puerto $AGENT_PORT")
fi

for esp_ip in "${ESP32_IPS[@]}"; do
    if ! ping -c 1 -W 1 "$esp_ip" &>/dev/null; then
        ISSUES+=("ESP32 $esp_ip no responde")
    fi
done

if [ "$ROS_LOCALHOST_ONLY" == "1" ] || [ "$ROS_LOCALHOST_ONLY" == "true" ]; then
    ISSUES+=("ROS_LOCALHOST_ONLY está bloqueando comunicación de red")
fi

# Verificar conectividad al router
GATEWAY=$(ip route | grep default | awk '{print $3}' | head -n 1)
if [ -n "$GATEWAY" ]; then
    if ! ping -c 1 -W 1 "$GATEWAY" &>/dev/null; then
        ISSUES+=("Router/Gateway ($GATEWAY) no responde")
    fi
else
    ISSUES+=("No se detectó gateway predeterminado")
fi

if [ ${#ISSUES[@]} -eq 0 ]; then
    echo -e "  ${GREEN}✅ No se detectaron problemas evidentes${NC}"
    echo -e "  ${CYAN}Si aún tienes problemas de comunicación:${NC}"
    echo -e "    ${CYAN}1. Verifica los logs del agente micro-ROS (ejecuta con flag -v6)${NC}"
    echo -e "    ${CYAN}2. Revisa los logs seriales de las ESP32s${NC}"
    echo -e "    ${CYAN}3. Usa tcpdump/Wireshark para capturar tráfico UDP en puerto $AGENT_PORT${NC}"
    echo -e "    ${CYAN}4. Verifica que AP Isolation esté DESACTIVADO en el router${NC}"
else
    echo -e "  ${RED}❌ Se detectaron los siguientes problemas:${NC}"
    for issue in "${ISSUES[@]}"; do
        echo -e "    ${RED}- $issue${NC}"
    done
    
    echo -e "\n  ${YELLOW}📋 Pasos sugeridos de corrección:${NC}"
    
    if [ "$UFW_STATUS" == "active" ]; then
        echo -e "    ${CYAN}1. Desactivar firewall temporalmente para pruebas:${NC}"
        echo -e "       ${WHITE}sudo ufw disable${NC}"
        echo -e "       ${WHITE}O agregar regla: sudo ufw allow $AGENT_PORT/udp${NC}"
    fi
    
    if ! ss -ulnp 2>/dev/null | grep -q ":$AGENT_PORT"; then
        echo -e "    ${CYAN}2. Iniciar el agente micro-ROS:${NC}"
        echo -e "       ${WHITE}ros2 run micro_ros_agent micro_ros_agent udp4 --port $AGENT_PORT -v6${NC}"
    fi
    
    # Verificar si hay problemas de router
    GATEWAY=$(ip route | grep default | awk '{print $3}' | head -n 1)
    if [ -n "$GATEWAY" ]; then
        if ! ping -c 1 -W 1 "$GATEWAY" &>/dev/null; then
            echo -e "    ${CYAN}3. Verificar router/gateway:${NC}"
            echo -e "       ${WHITE}- Reiniciar router${NC}"
            echo -e "       ${WHITE}- Verificar cables de red o conexión WiFi${NC}"
            echo -e "       ${WHITE}- Revisar configuración DHCP y AP Isolation${NC}"
        fi
    fi
fi

echo -e "\n${CYAN}=====================================${NC}"
echo -e "${CYAN} Diagnóstico completado${NC}"
echo -e "${CYAN}=====================================${NC}"
