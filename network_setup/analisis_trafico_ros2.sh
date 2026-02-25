#!/bin/bash
# analisis_trafico_ros2.sh
# Herramienta para medir rendimiento de ROS 2 y diagnosticar latencia en la subred

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m'

echo -e "${CYAN}===============================================${NC}"
echo -e "${CYAN}   Análisis de Rendimiento ROS 2 - Subred      ${NC}"
echo -e "${CYAN}===============================================${NC}"

# 0. Verificación de Variables de Entorno
echo -e "\n${YELLOW}[0/5] Verificando Variables de Entorno ROS 2...${NC}"
echo -e "  ${WHITE}ROS_DOMAIN_ID:${NC} ${ROS_DOMAIN_ID:-0} (Default: 0)"

if [ "$RMW_IMPLEMENTATION" == "rmw_cyclonedds_cpp" ]; then
    echo -e "  ${WHITE}RMW_IMPLEMENTATION:${NC} ${GREEN}${RMW_IMPLEMENTATION}${NC} (Optimizado para WiFi)"
elif [ -z "$RMW_IMPLEMENTATION" ]; then
    echo -e "  ${WHITE}RMW_IMPLEMENTATION:${NC} ${YELLOW}No definida${NC} (Usando FastDDS por defecto)"
else
    echo -e "  ${WHITE}RMW_IMPLEMENTATION:${NC} ${WHITE}${RMW_IMPLEMENTATION}${NC}"
fi

echo -e "  ${WHITE}ROS_AUTOMATIC_DISCOVERY_RANGE:${NC} ${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

# 1. Escaneo de dominios activos (DDS Discovery Scan)
echo -e "\n${YELLOW}[1/5] Escaneando Dominios ROS 2 activos en la red...${NC}"
echo -e "  (Buscando tráfico en puertos estándar DDS 7400-7500)"

active_domains=()
# Escaneamos puertos base de descubrimiento para dominios 0 a 100
# Puerto = 7400 + (DomainID * 250)
if command -v ss &>/dev/null; then
    # Usamos ss para ver qué puertos UDP están recibiendo/enviando
    dds_ports=$(ss -unlp | grep -oP ':(74\d\d|7[5-9]\d\d|8\d\d\d)\s' | tr -d ' :' | sort -u)
    
    for port in $dds_ports; do
        if [ "$port" -ge 7400 ] && [ "$port" -le 32000 ]; then
            # Fórmula inversa aproximada: (Port - 7400) / 250
            domain_id=$(( (port - 7400) / 250 ))
            if [[ ! " ${active_domains[@]} " =~ " ${domain_id} " ]]; then
                active_domains+=($domain_id)
            fi
        fi
    done
else
    echo -e "  ${RED}Error: Herramienta 'ss' no encontrada. No se pudo escanear dominios.${NC}"
fi

if [ ${#active_domains[@]} -eq 0 ]; then
    echo -e "  ${WHITE}No se detectaron dominios activos además del actual (ID: ${ROS_DOMAIN_ID:-0}).${NC}"
else
    echo -e "  ${GREEN}Dominios detectados con actividad DDS:${NC} ${active_domains[*]}"
    if [ ${#active_domains[@]} -gt 1 ]; then
        echo -e "  ${YELLOW}⚠️  Múltiples dominios detectados. Esto puede generar congestión en el router WiFi.${NC}"
    fi
fi

# 2. Medición de Ancho de Banda (Bandwidth)
echo -e "\n${YELLOW}[2/5] Medición de Ancho de Banda en Tópicos Activos...${NC}"
topics=$(timeout 2 ros2 topic list 2>/dev/null)

if [ -z "$topics" ]; then
    echo -e "  ${WHITE}No hay tópicos activos para medir.${NC}"
else
    echo -e "  Tópicos disponibles: "
    echo "$topics" | head -n 5 | sed 's/^/    /'
    
    # Elegir el primer tópico que no sea de sistema para una prueba rápida
    test_topic=$(echo "$topics" | grep -v "/parameter_events\|/rosout" | head -n 1)
    
    if [ -n "$test_topic" ]; then
        echo -e "\n  Prueba de ancho de banda en: ${CYAN}$test_topic${NC} (3 segundos)..."
        timeout 4 ros2 topic bw "$test_topic" 2>/dev/null | grep -E "average|BW" || echo "  No se recibió suficiente data para medir BW."
    else
        echo -e "  Inicia un publicador en otra terminal para medir ancho de banda real."
    fi
fi

# 3. Diagnóstico de Causas de Latencia
echo -e "\n${YELLOW}[3/5] Análisis de Causas de Latencia Física...${NC}"
GATEWAY=$(ip route | grep default | awk '{print $3}' | head -n 1)

if [ -n "$GATEWAY" ]; then
    echo -e "  Realizando ráfaga de pings a $GATEWAY para medir estabilidad..."
    # Usamos un ping normal pero procesamos mejor el output
    ping_result=$(ping -c 5 -W 2 "$GATEWAY")
    
    avg_latency=$(echo "$ping_result" | tail -1 | awk -F '/' '{print $5}')
    loss=$(echo "$ping_result" | grep -oP '\d+(?=% packet loss)')
    
    echo -e "  ${WHITE}Latencia Promedio:${NC} ${avg_latency:-N/A}ms"
    echo -e "  ${WHITE}Pérdida de Paquetes:${NC} ${loss:-0}%"
    
    if [[ -n "$avg_latency" ]]; then
        if (( $(echo "$avg_latency > 150" | bc -l) )); then
            echo -e "  ${RED}❌ CAUSA DETECTADA: Saturación del medio inalámbrico o mala señal.${NC}"
            echo -e "     Tu enlace base es demasiado lento para ROS 2."
        elif [[ "$loss" -gt 0 ]]; then
            echo -e "  ${RED}❌ CAUSA DETECTADA: Pérdida de paquetes (Interferencias WiFi).${NC}"
            echo -e "     DDS (UDP) es muy sensible a la pérdida de paquetes.${NC}"
        else
            echo -e "  ${GREEN}✅ Enlace base físicamente estable.${NC}"
        fi
    fi
fi

# Verificación de compatibilidad con WiFi 6 (802.11ax)
echo -e "  Verificando hardware de red del equipo..."
if grep -qi "microsoft" /proc/version 2>/dev/null; then
    # WSL: usar netsh.exe de Windows
    if command -v netsh.exe &>/dev/null; then
        wifi_types=$(netsh.exe wlan show drivers 2>/dev/null)
        if echo "$wifi_types" | grep -qi "802.11ax"; then
            echo -e "  ${GREEN}✅ Tarjeta de red COMPATIBLE con WiFi 6 (802.11ax).${NC}"
        elif echo "$wifi_types" | grep -qi "802.11ac\|802.11n"; then
            echo -e "  ${YELLOW}⚠️  Tarjeta de red NO COMPATIBLE con WiFi 6 (Solo soporta hasta WiFi 5 / AC o inferior).${NC}"
            echo -e "     Limitación de hardware de la PC: No aprovecharás el 100% del router AX12."
        fi
    fi
else
    # GNU/Linux nativo
    if command -v iw &>/dev/null; then
        if iw list 2>/dev/null | grep -qiE "HE MAC|HE PHY|802.11ax"; then
            echo -e "  ${GREEN}✅ Tarjeta de red COMPATIBLE con WiFi 6 (802.11ax).${NC}"
        else
            echo -e "  ${YELLOW}⚠️  Tarjeta de red NO COMPATIBLE con WiFi 6 (Solo soporta hasta WiFi 5 / AC o inferior).${NC}"
            echo -e "     Limitación de hardware de la PC: No aprovecharás el 100% del router AX12."
        fi
    fi
fi

# 4. Recomendaciones de Optimización
echo -e "\n${YELLOW}[4/5] Guía de Mitigación de Latencia:${NC}"

echo -e "  ${WHITE}1. Cambiar a RMW CycloneDDS:${NC}"
current_rmw="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
echo -e "     ${WHITE}Estado Actual:${NC} $current_rmw"

if [ "$current_rmw" == "rmw_cyclonedds_cpp" ]; then
    echo -e "     ${GREEN}✅ Configuración correcta para WiFi.${NC}"
else
    echo -e "     ${YELLOW}⚠️  Recomendado: Cambiar a rmw_cyclonedds_cpp para mejorar estabilidad.${NC}"
    # Verificar si está en .bashrc pero no cargado
    if grep -q "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ~/.bashrc; then
        echo -e "     ${CYAN}👉 Info: Ya está en tu .bashrc pero no se ha cargado en esta terminal.${NC}"
        echo -e "     Ejecuta: ${WHITE}source ~/.bashrc${NC}"
    else
        echo -e "     Agrega a .bashrc: ${CYAN}export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp${NC}"
    fi
fi

echo -e "\n  ${WHITE}2. Usar Discovery Server (Recomendado):${NC}"
echo -e "     En redes WiFi con muchos dominios o usuarios, el multicast falla."
echo -e "     Consulta: ${CYAN}network_setup/ROS2_NETWORK_CONFIG.md${NC} (sección Fast DDS Server)"

echo -e "\n  ${WHITE}3. Desactivar Smart Connect:${NC}"
echo -e "     En routers WiFi 6 (Archer AX12), el salto entre 2.4 y 5GHz causa picos de lag."

echo -e "\n  ${WHITE}4. MTU y Fragmentación (Imágenes o mapas densos):${NC}"
echo -e "     El WiFi tiene un límite de envío (MTU) de 1500 bytes. Mensajes grandes se fragmentan."
echo -e "     Si una parte choca en el aire, se pierde el mensaje completo, causando lag."
echo -e "     ${CYAN}Consejos para evitarlo:${NC}"
echo -e "     ${WHITE}a)${NC} NUNCA envíes imágenes RAW por WiFi, usa compresión (paquete image_transport)."
echo -e "     ${WHITE}b)${NC} Si hay problemas, amplía los buffers UDP de Linux (comando sysctl net.core.rmem_max)."
echo -e "     ${WHITE}c)${NC} Mantén CycloneDDS activo, ya que gestiona mejor los paquetes grandes que FastDDS."

echo -e "\n${CYAN}===============================================${NC}"
echo -e "${CYAN}          Análisis de Red Completado           ${NC}"
echo -e "${CYAN}===============================================${NC}"
