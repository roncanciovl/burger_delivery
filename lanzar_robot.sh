#!/bin/bash
# Script para lanzar el robot en Linux/WSL

# Cargar el entorno de ROS 2 local
if [ -f "/home/roncanciovl/ros2_ws/install/setup.bash" ]; then
    echo "Cargando entorno del workspace..."
    source /home/roncanciovl/ros2_ws/install/setup.bash
else
    echo "Error: No se encontró ~/ros2_ws/install/setup.bash. Asegúrate de haber compilado primero."
    exit 1
fi

# Lanzar la visualización
echo "Lanzando visualización del robot..."
ros2 launch burger_description display.launch.py &

# Esperar un poco para que rviz abra
sleep 5

# Lanzar rqt para controles
echo "Lanzando rqt..."
rqt
