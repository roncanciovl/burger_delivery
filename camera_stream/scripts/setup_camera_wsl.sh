#!/bin/bash
###############################################################################
# setup_camera_wsl.sh
# Configura el acceso a la cámara USB dentro de WSL2
# 
# Requisitos previos:
#   - Windows 11 con WSL2 (Ubuntu 24.04)
#   - usbipd-win instalado en Windows (ver instrucciones abajo)
#
# Uso: bash setup_camera_wsl.sh
###############################################################################

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   Configuración de Cámara USB en WSL2                       ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

# ============================================================================
# PASO 1: Verificar que estamos en WSL2
# ============================================================================
echo -e "${YELLOW}[1/4] Verificando entorno WSL2...${NC}"
if grep -qi microsoft /proc/version 2>/dev/null; then
    echo -e "${GREEN}  ✓ Estamos en WSL2${NC}"
    echo "    Kernel: $(uname -r)"
else
    echo -e "${RED}  ✗ Este script debe ejecutarse dentro de WSL2${NC}"
    exit 1
fi

# ============================================================================
# PASO 2: Instalar dependencias de Linux para USB y video
# ============================================================================
echo ""
echo -e "${YELLOW}[2/4] Instalando dependencias de USB y video en WSL...${NC}"

# v4l-utils para verificar dispositivos de video
sudo apt-get update -qq
sudo apt-get install -y -qq v4l-utils linux-tools-generic hwdata > /dev/null 2>&1 || true

# Intentar instalar linux-tools para el kernel específico de WSL
KERNEL_VERSION=$(uname -r)
sudo apt-get install -y -qq "linux-tools-${KERNEL_VERSION}" 2>/dev/null || {
    echo -e "${YELLOW}  ⚠ linux-tools para kernel WSL no disponible (es normal)${NC}"
    echo "    Se usará la versión genérica"
}

# usbutils para lsusb
sudo apt-get install -y -qq usbutils > /dev/null 2>&1

echo -e "${GREEN}  ✓ Dependencias instaladas${NC}"

# ============================================================================
# PASO 3: Cargar módulos del kernel para video
# ============================================================================
echo ""
echo -e "${YELLOW}[3/4] Cargando módulos del kernel para cámara USB...${NC}"

# Cargar uvcvideo (driver de cámaras USB)
if sudo modprobe uvcvideo 2>/dev/null; then
    echo -e "${GREEN}  ✓ Módulo uvcvideo cargado${NC}"
else
    echo -e "${RED}  ✗ No se pudo cargar uvcvideo${NC}"
    echo "    Tu kernel WSL puede no tener soporte USB Video."
    echo "    Verifica con: modinfo uvcvideo"
    exit 1
fi

# Cargar videobuf2 si es necesario
sudo modprobe videobuf2_vmalloc 2>/dev/null || true
sudo modprobe videobuf2_memops 2>/dev/null || true
sudo modprobe videobuf2_common 2>/dev/null || true
sudo modprobe videobuf2_v4l2 2>/dev/null || true

echo -e "${GREEN}  ✓ Módulos de video cargados${NC}"

# ============================================================================
# PASO 4: Verificar estado
# ============================================================================
echo ""
echo -e "${YELLOW}[4/4] Verificando estado...${NC}"

# Verificar módulos cargados
if lsmod | grep -q uvcvideo; then
    echo -e "${GREEN}  ✓ uvcvideo está activo en el kernel${NC}"
else
    echo -e "${RED}  ✗ uvcvideo no se cargó correctamente${NC}"
fi

# Verificar si hay dispositivos de video
if ls /dev/video* 2>/dev/null; then
    echo -e "${GREEN}  ✓ Dispositivos de video detectados:${NC}"
    ls -la /dev/video*
    echo ""
    echo -e "${GREEN}  Información de la cámara:${NC}"
    v4l2-ctl --list-devices 2>/dev/null || true
else
    echo -e "${YELLOW}  ⚠ No hay dispositivos de video aún${NC}"
    echo ""
    echo -e "${BLUE}══════════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}  SIGUIENTE PASO: Conectar la cámara desde Windows${NC}"
    echo -e "${BLUE}══════════════════════════════════════════════════════════════${NC}"
    echo ""
    echo "  Abre PowerShell como ADMINISTRADOR en Windows y ejecuta:"
    echo ""
    echo -e "${GREEN}  1. Ver dispositivos USB disponibles:${NC}"
    echo "     usbipd list"
    echo ""
    echo -e "${GREEN}  2. Busca tu cámara en la lista (ej: 'USB Camera' o 'Integrated Webcam')${NC}"
    echo "     Anota el BUSID (ej: 1-3)"
    echo ""
    echo -e "${GREEN}  3. Conectar la cámara a WSL:${NC}"
    echo "     usbipd bind --busid <BUSID>"
    echo "     usbipd attach --wsl --busid <BUSID>"
    echo ""
    echo -e "${GREEN}  4. Verificar en WSL (en otra terminal):${NC}"
    echo "     ls /dev/video*"
    echo "     v4l2-ctl --list-devices"
    echo ""
fi

echo ""
echo -e "${BLUE}══════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  INSTALACIÓN DE usbipd-win EN WINDOWS (si no lo tienes)${NC}"
echo -e "${BLUE}══════════════════════════════════════════════════════════════${NC}"
echo ""
echo "  Abre PowerShell como ADMINISTRADOR y ejecuta:"
echo ""
echo "     winget install --interactive --exact dorssel.usbipd-win"
echo ""
echo "  Después reinicia Windows y ejecuta este script de nuevo."
echo ""
