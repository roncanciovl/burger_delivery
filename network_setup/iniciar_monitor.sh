#!/bin/bash
# ==============================================================================
# iniciar_monitor.sh
# Lanzador de la Interfaz Web del Monitor de Red Híbrido (ROS 2 + Router + Micro-ROS)
# ==============================================================================

PORT=${1:-8080}
HOST="127.0.0.1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVER_SCRIPT="$SCRIPT_DIR/monitor_red/server.py"

CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${CYAN}================================================================${NC}"
echo -e "${CYAN}   🚀 Iniciando Monitor de Red Híbrido (ROS 2 & Router AX12)     ${NC}"
echo -e "${CYAN}================================================================${NC}"

if ! command -v python3 &>/dev/null; then
    echo -e "${YELLOW}Error: Python 3 no está instalado.${NC}"
    exit 1
fi

URL="http://${HOST}:${PORT}"
echo -e "  ${GREEN}🌐 URL del Dashboard:${NC} ${CYAN}${URL}${NC}"
echo -e "  ${YELLOW}💡 Abriendo navegador...${NC}\n"

# Intentar abrir navegador según el entorno
if grep -qi "microsoft" /proc/version 2>/dev/null; then
    # Entorno WSL2
    if command -v wslview &>/dev/null; then
        wslview "$URL" 2>/dev/null &
    elif command -v cmd.exe &>/dev/null; then
        cmd.exe /c start "$URL" 2>/dev/null &
    fi
elif command -v xdg-open &>/dev/null; then
    # Linux Nativo
    xdg-open "$URL" 2>/dev/null &
fi

# Ejecutar el servidor web en primer plano
python3 "$SERVER_SCRIPT" --host "$HOST" --port "$PORT"
