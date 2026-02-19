#!/bin/bash
# diagnostico_wifi.sh
# Script de diagnóstico para verificar la calidad y estado de la conexión WiFi
# Optimizado para entornos Linux nativos y WSL (Windows Subsystem for Linux)
# Uso: ./diagnostico_wifi.sh

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color

echo -e "${CYAN}===============================================${NC}"
echo -e "${CYAN}   Diagnóstico de Red WiFi - Burger Delivery   ${NC}"
echo -e "${CYAN}===============================================${NC}"

# Detectar si estamos en WSL
IS_WSL=false
if grep -qi "microsoft" /proc/version 2>/dev/null; then
    IS_WSL=true
fi

# ======================================
# 1. Verificar Adaptador WiFi / Red
# ======================================
echo -e "\n${YELLOW}[1/6] Verificando Adaptador de Red...${NC}"

# Intentar encontrar interfaz wifi real primero
WIFI_INTERFACE=$(ip link show | grep -i "wlan\|wifi" | head -n 1 | awk -F: '{print $2}' | xargs)

if [ -z "$WIFI_INTERFACE" ]; then
    if [ "$IS_WSL" = true ]; then
        # En WSL, usamos la interfaz que tiene la ruta por defecto
        WIFI_INTERFACE=$(ip route | grep default | awk '{print $5}' | head -n 1)
        echo -e "  ${WHITE}Entorno:${NC} WSL Detectado"
        echo -e "  ${YELLOW}⚠️  En WSL las tarjetas WiFi se ven como Ethernet virtual.${NC}"
        echo -e "  Usando interfaz puente: ${WHITE}$WIFI_INTERFACE${NC}"
    else
        echo -e "  ${RED}❌ No se encontró ningún adaptador WiFi.${NC}"
        echo -e "  Verifica que el hardware esté conectado o los drivers instalados."
        exit 1
    fi
fi

if [ -z "$WIFI_INTERFACE" ]; then
    echo -e "  ${RED}❌ No se detectó ninguna interfaz de red activa.${NC}"
    exit 1
fi

STATE=$(cat /sys/class/net/$WIFI_INTERFACE/operstate 2>/dev/null || ip addr show $WIFI_INTERFACE | grep -oP 'state \K\w+')
echo -e "  ${WHITE}Interfaz:${NC} $WIFI_INTERFACE"
if [[ "$STATE" == "up" || "$STATE" == "UNKNOWN" ]]; then
    echo -e "  ${GREEN}✅ Estado: ACTIVO${NC}"
else
    echo -e "  ${RED}❌ Estado: $STATE (Desconectado)${NC}"
    echo -e "  Intenta activar con: sudo ip link set $WIFI_INTERFACE up"
fi

# ======================================
# 2. Calidad de Señal y SSID
# ======================================
echo -e "\n${YELLOW}[2/6] Analizando Calidad de Señal...${NC}"

if [ "$IS_WSL" = true ]; then
    echo -e "  ${YELLOW}⚠️  WSL no puede acceder directamente al hardware WiFi.${NC}"
    echo -e "  ${CYAN}👉 Para ver nivel de señal real, ejecuta en Windows:${NC}"
    echo -e "     ${WHITE}.\\network_setup\\diagnostico_wifi.ps1${NC}"
else
    if command -v nmcli &> /dev/null; then
        WIFI_INFO=$(nmcli -t -f ACTIVE,SSID,SIGNAL,BARS,RATE dev wifi | grep '^yes')
        if [ -n "$WIFI_INFO" ]; then
            SSID=$(echo "$WIFI_INFO" | cut -d':' -f2)
            SIGNAL=$(echo "$WIFI_INFO" | cut -d':' -f3)
            BARS=$(echo "$WIFI_INFO" | cut -d':' -f4)
            RATE=$(echo "$WIFI_INFO" | cut -d':' -f5)
            
            echo -e "  ${WHITE}SSID Conectado:${NC} $SSID"
            echo -e "  ${WHITE}Velocidad:${NC} $RATE"
            
            if [ "$SIGNAL" -ge 75 ]; then
                echo -e "  ${GREEN}✅ Señal: $SIGNAL% ($BARS) - Excelente${NC}"
            elif [ "$SIGNAL" -ge 50 ]; then
                echo -e "  ${YELLOW}⚠️  Señal: $SIGNAL% ($BARS) - Media (Puede afectar latencia)${NC}"
            else
                echo -e "  ${RED}❌ Señal: $SIGNAL% ($BARS) - Débil (Riesgo de desconexión)${NC}"
            fi
        else
            echo -e "  ${RED}❌ No conectado a ninguna red WiFi física.${NC}"
        fi
    elif command -v iwconfig &> /dev/null; then
        IW_OUTPUT=$(iwconfig $WIFI_INTERFACE 2>/dev/null)
        SSID=$(echo "$IW_OUTPUT" | grep -oP 'ESSID:"\K[^"]+')
        QUALITY=$(echo "$IW_OUTPUT" | grep -oP 'Link Quality=\K\d+/\d+')
        
        echo -e "  ${WHITE}SSID:${NC} $SSID"
        echo -e "  ${WHITE}Calidad de Link:${NC} $QUALITY"
    else
        echo -e "  ${WHITE}ℹ️  nmcli/iwconfig no instalados o no compatibles con esta interfaz.${NC}"
    fi
fi

# ======================================
# 3. Escaneo de Canales (Congestión)
# ======================================
echo -e "\n${YELLOW}[3/6] Escaneando Redes Cercanas...${NC}"
if [ "$IS_WSL" = true ]; then
    echo -e "  ${WHITE}ℹ️  Escaneo no disponible en WSL.${NC}"
else
    if command -v nmcli &> /dev/null; then
        nmcli -f SSID,CHAN,SIGNAL,SECURITY dev wifi list | head -n 6 | sed 's/^/    /'
    else
        echo -e "  ${WHITE}ℹ️  nmcli no disponible para escaneo.${NC}"
    fi
fi

# ======================================
# 4. Pruebas de Latencia y Conectividad
# ======================================
echo -e "\n${YELLOW}[4/6] Pruebas de Conectividad y Latencia...${NC}"

# Gateway
GATEWAY=$(ip route | grep default | awk '{print $3}' | head -n 1)
if [ -n "$GATEWAY" ]; then
    echo -n "  Ping al Gateway ($GATEWAY): "
    PING_GW=$(ping -c 3 -W 2 "$GATEWAY" 2>/dev/null)
    if [ $? -eq 0 ]; then
        LATENCY=$(echo "$PING_GW" | tail -1 | awk -F '/' '{print $5}')
        echo -e "${GREEN}OK (${LATENCY}ms)${NC}"
        if (( $(echo "$LATENCY > 50" | bc -l) )); then
            echo -e "  ${YELLOW}⚠️  Latencia alta hacia el router (>50ms).${NC}"
        fi
    else
        echo -e "${RED}FALLIDO${NC}"
        if [ "$IS_WSL" = true ]; then
            echo -e "  ${WHITE}Nota: En WSL 'mirrored mode', el gateway puede no responder al ping.${NC}"
        fi
    fi
else
    echo -e "  ${RED}❌ No se detectó Gateway.${NC}"
fi

# Internet
echo -n "  Ping Internet (8.8.8.8): "
if ping -c 2 -W 2 8.8.8.8 &>/dev/null; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}SIN ACCESO WAN${NC}"
fi

# DNS
echo -n "  Resolución DNS (google.com): "
if getent hosts google.com &>/dev/null || nslookup google.com &>/dev/null || host google.com &>/dev/null; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FALLIDO${NC}"
fi

# ======================================
# 5. Configuración IP y DNS
# ======================================
echo -e "\n${YELLOW}[5/6] Configuración de Red Local:${NC}"
IP_ADDR=$(ip -4 addr show $WIFI_INTERFACE | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -n 1)
echo -e "  ${WHITE}IP Local:${NC} $IP_ADDR"

if [ -f /etc/resolv.conf ]; then
    DNS_SERVERS=$(grep nameserver /etc/resolv.conf | awk '{print $2}' | tr '\n' ' ')
    echo -e "  ${WHITE}DNS Servidores:${NC} $DNS_SERVERS"
fi

# ======================================
# 6. Recomendaciones
# ======================================
echo -e "\n${YELLOW}[6/6] Resumen y Recomendaciones:${NC}"

if [ "$IS_WSL" = true ]; then
    echo -e "  ${WHITE}● ESTÁS EN WSL: El diagnóstico físico debe hacerse desde Windows.${NC}"
    echo -e "  ${WHITE}● Verificado: Tienes conexión IP activa a través de $WIFI_INTERFACE.${NC}"
fi

if [[ -n "$LATENCY" ]]; then
    if (( $(echo "$LATENCY > 100" | bc -l) )); then
         echo -e "  ${RED}● LATENCIA CRÍTICA: Comunicación ROS2 inestable (>100ms).${NC}"
    fi
fi

echo -e "\n${CYAN}===============================================${NC}"
echo -e "${CYAN}             Diagnóstico Finalizado            ${NC}"
echo -e "${CYAN}===============================================${NC}"
