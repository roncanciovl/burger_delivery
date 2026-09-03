#!/bin/bash
# ROS 2 Network Test Script
# Verifica que ESTA máquina esté correctamente configurada para participar en la
# red ROS 2 distribuida del proyecto. No depende de nombres de interfaz ni de IP
# fijas: sirve igual en Ubuntu nativo y en WSL2 reflejado.
#
# Devuelve 0 si la configuración local es utilizable, 1 si hay un fallo que
# impide comunicarse con otros computadores.

DOMAIN_ESPERADO="${ROS_DOMAIN_ID_ESPERADO:-0}"
FALLOS=0

echo "=========================================="
echo "ROS 2 Network Configuration Test"
echo "=========================================="
echo ""

# 1. Variables de entorno
echo "1. Variables de entorno de ROS 2:"
echo "   ROS_DISTRO: ${ROS_DISTRO:-NO DEFINIDO (¿falta 'source /opt/ros/<distro>/setup.bash'?)}"
echo "   ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-NO DEFINIDO (por defecto: 0)}"
if [ "${ROS_DOMAIN_ID:-0}" != "$DOMAIN_ESPERADO" ]; then
    echo "   ⚠️  El proyecto usa el dominio $DOMAIN_ESPERADO. Todas las máquinas que deban"
    echo "      verse entre sí necesitan el MISMO valor."
fi
echo "   ROS_AUTOMATIC_DISCOVERY_RANGE: ${ROS_AUTOMATIC_DISCOVERY_RANGE:-NO DEFINIDO (por defecto: SUBNET)}"
case "${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}" in
    SUBNET) ;;
    *) echo "   ✗ Debe ser SUBNET para descubrir nodos de otros computadores."; FALLOS=$((FALLOS+1)) ;;
esac
if [ -n "$ROS_LOCALHOST_ONLY" ]; then
    echo "   ✗ ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY está definida y es obsoleta;"
    echo "     puede aislar este equipo. Elimínala de ~/.bashrc."
    FALLOS=$((FALLOS+1))
fi
echo "   RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-NO DEFINIDO (por defecto: rmw_fastrtps_cpp)}"
if [ "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" ]; then
    echo "   ⚠️  El proyecto usa rmw_cyclonedds_cpp. Equipos con RMW distinto NO se comunican."
fi
echo ""

# 2. Perfil DDS
echo "2. Perfil CycloneDDS:"
if [ -z "$CYCLONEDDS_URI" ]; then
    echo "   ⚠️  CYCLONEDDS_URI no está definida: se usarán los valores por defecto de DDS."
else
    PERFIL="${CYCLONEDDS_URI#file://}"
    if [ -r "$PERFIL" ]; then
        echo "   ✓ Perfil legible: $PERFIL"
        # Lee el atributo name= ignorando los bloques comentados del perfil.
        IFACE_FIJA=$(python3 - "$PERFIL" <<'PYEOF' 2>/dev/null
import sys, xml.etree.ElementTree as ET
try:
    raiz = ET.parse(sys.argv[1]).getroot()
except ET.ParseError:
    sys.exit(0)
for elemento in raiz.iter():
    if elemento.tag.rsplit('}', 1)[-1] == 'NetworkInterface':
        nombre = elemento.get('name')
        if nombre:
            print(nombre)
            break
PYEOF
)
        if [ -n "$IFACE_FIJA" ]; then
            if ip -brief addr | awk '{print $1}' | grep -qx "$IFACE_FIJA"; then
                echo "   ✓ Interfaz fijada en el perfil: $IFACE_FIJA (existe)"
            else
                echo "   ✗ El perfil fija la interfaz '$IFACE_FIJA', que NO existe aquí."
                echo "     CycloneDDS abortará todo nodo. Usa autodetermine=\"true\" o el nombre real."
                FALLOS=$((FALLOS+1))
            fi
        else
            echo "   ✓ Selección automática de interfaz (portable entre equipos)"
        fi
    else
        echo "   ✗ CYCLONEDDS_URI apunta a un archivo ilegible: $PERFIL"
        FALLOS=$((FALLOS+1))
    fi
fi
echo ""

# 3. Red local
echo "3. Información de red:"
ip -brief addr | grep -v '^lo' | sed 's/^/   /'
echo "   IP principal: $(hostname -I | awk '{print $1}')"
echo "   Gateway: $(ip route | grep default | awk '{print $3}' | head -n 1)"
if grep -qi microsoft /proc/version 2>/dev/null; then
    MODO=$(wslinfo --networking-mode 2>/dev/null || echo "desconocido")
    echo "   WSL networking mode: $MODO"
    if [ "$MODO" != "mirrored" ]; then
        echo "   ✗ En modo '$MODO' los nodos de WSL no son visibles en la red física."
        echo "     Configura networkingMode=mirrored en %USERPROFILE%\\.wslconfig."
        FALLOS=$((FALLOS+1))
    fi
fi
echo ""

# 4. Instalación de ROS 2
echo "4. Instalación de ROS 2:"
if command -v ros2 &> /dev/null; then
    echo "   ✓ Comando ros2 encontrado"
else
    echo "   ✗ ros2 no está en el PATH"
    exit 1
fi
echo ""

# 5. Prueba de arranque real del middleware
echo "5. Arranque del middleware (sin daemon):"
SALIDA=$(timeout 20 ros2 topic list --no-daemon 2>&1)
if echo "$SALIDA" | grep -q "/parameter_events"; then
    echo "   ✓ El RMW crea nodos correctamente"
    echo "$SALIDA" | sed 's/^/     /'
else
    echo "   ✗ No fue posible crear un nodo. La configuración local está rota:"
    echo "$SALIDA" | head -5 | sed 's/^/     /'
    FALLOS=$((FALLOS+1))
fi
echo ""

# 6. Nodos y tópicos vistos a través del daemon
echo "6. Grafo ROS 2 visible (con daemon):"
timeout 10 ros2 node list 2>/dev/null | sed 's/^/   /' || \
    echo "   (sin nodos, o daemon bloqueado: ver TROUBLESHOOTING.md)"
echo ""

# 7. Puertos DDS del dominio en uso
DOMINIO="${ROS_DOMAIN_ID:-0}"
PUERTO_BASE=$((7400 + 250 * DOMINIO))
echo "7. Puertos DDS del dominio $DOMINIO (base $PUERTO_BASE):"
if command -v ss &> /dev/null; then
    ENCONTRADOS=$(ss -uln | awk -v p="$PUERTO_BASE" 'NR>1 {split($4,a,":"); puerto=a[length(a)]+0; if (puerto >= p && puerto <= p+249) print "   " $4}' | sort -u | head -8)
    if [ -n "$ENCONTRADOS" ]; then
        echo "$ENCONTRADOS"
    else
        echo "   (ninguno: normal si no hay nodos ROS 2 ejecutándose en este equipo)"
    fi
else
    echo "   ss no disponible"
fi
echo ""

echo "=========================================="
if [ "$FALLOS" -eq 0 ]; then
    echo "Configuración local correcta."
else
    echo "Se encontraron $FALLOS problema(s) que impiden la comunicación distribuida."
fi
echo "=========================================="
echo ""
echo "Para probar contra otro computador (ambos con ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}):"
echo "  1. Multicast:  PC1 'ros2 multicast receive'  |  PC2 'ros2 multicast send'"
echo "     (usa el grupo 225.0.0.1:49150, no los puertos DDS del dominio)"
echo "  2. Tópicos:    PC1 'ros2 topic pub /hello std_msgs/String \"data: hola\"'"
echo "                 PC2 'ros2 topic echo /hello'"
echo ""
echo "Diagnóstico detallado: network_setup/ROS2_NETWORK_CONFIG.md"

exit $(( FALLOS > 0 ))
