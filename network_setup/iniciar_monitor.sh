#!/bin/bash
# ==============================================================================
# iniciar_monitor.sh
# Lanzador de la Interfaz Web del Monitor de Red Híbrido (ROS 2 + Router + Micro-ROS)
# ==============================================================================

PORT=${1:-8080}
HOST="0.0.0.0"
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

URL="http://localhost:${PORT}"
echo -e "  ${GREEN}🌐 URL del Dashboard:${NC} ${CYAN}${URL}${NC}"
echo -e "  ${YELLOW}💡 Abriendo navegador...${NC}\n"


# Limpiar posibles instancias anteriores huérfanas
pkill -f "monitor_red/server.py" 2>/dev/null
sleep 0.2

# Ejecutar el servidor web en primer plano (server.py abre automáticamente el navegador en el puerto activo)
python3 "$SERVER_SCRIPT" --host "$HOST" --port "$PORT"



