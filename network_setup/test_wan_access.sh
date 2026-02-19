#!/bin/bash

# Script para detectar bloqueos de acceso a la WAN (Internet)
# Útil para diagnosticar si el router o un firewall están bloqueando la salida

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}=== Verificación de Acceso WAN (Internet) ===${NC}\n"

# 1. Verificar Interfaz de Red y Gateway Local
echo -n "[1/4] Verificando conexión al Gateway (Router)... "
GATEWAY=$(ip route | grep default | awk '{print $3}' | head -n 1)

if [ -z "$GATEWAY" ]; then
    echo -e "${RED}FALLIDO: No se detectó una ruta por defecto (Gateway).${NC}"
else
    if ping -c 1 -W 2 "$GATEWAY" > /dev/null 2>&1; then
        echo -e "${GREEN}OK ($GATEWAY responde)${NC}"
    else
        echo -e "${RED}FALLIDO: El Gateway ($GATEWAY) no responde al ping. Posible desconexión física o AP Isolation.${NC}"
    fi
fi

# 2. Verificar Conectividad por IP (Evita problemas de DNS)
echo -n "[2/4] Verificando acceso WAN por IP (Google DNS 8.8.8.8)... "
if ping -c 1 -W 2 8.8.8.8 > /dev/null 2>&1; then
    echo -e "${GREEN}OK (Conexión directa a Internet detectada)${NC}"
else
    echo -e "${RED}BLOQUEADO: No hay salida a IPs externas. Posible bloqueo en el Router o Firewall.${NC}"
fi

# 3. Verificar Resolución DNS
echo -n "[3/4] Verificando resolución DNS (google.com)... "
if getent hosts google.com > /dev/null 2>&1 || host google.com > /dev/null 2>&1 || nslookup google.com > /dev/null 2>&1; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}FALLIDO: No se pueden resolver nombres. Revisa /etc/resolv.conf o DNS del Router.${NC}"
fi

# 4. Verificar Bloqueo de Puertos Comunes (HTTP/HTTPS)
echo -n "[4/4] Verificando salida por puerto 443 (HTTPS)... "
if curl -s --connect-timeout 5 https://www.google.com > /dev/null 2>&1; then
    echo -e "${GREEN}OK${NC}"
else
    echo -e "${RED}BLOQUEADO/LENTO: No hay respuesta HTTPS. Posible inspección de paquetes o bloqueo de puertos.${NC}"
fi

echo -e "\n${YELLOW}=== Resumen de Diagnóstico ===${NC}"
if [ $? -eq 0 ]; then
    echo "Si los pasos 2 y 4 fallan pero el 1 es OK, tu Router tiene desactivado el acceso WAN para este dispositivo."
    echo "Revisa la configuración de 'Control Parental' o 'Lista Negra' en tu TP-Link Archer AX12."
fi
