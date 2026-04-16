# PRUEBAS DEL ROBOT MEDIANTE COMANDOS (CLI)

Basado en la plataforma usada en el repositorio de GitHub de control avanzado logístico hemos integrado a tu workspace local un pequeño nodo Cliente enfocado en testeo unitario y validación de hardware directo en la consola. Este CLI bypassa las rutinas de Burger Delivery interactuando de puente directo a **MoveIt 2** y sus *ActionServers*.

**Ubicación del Script de Prueba**: `scripts/test_kinova_pose.py`

---

## 1. ¿Cómo utilizarlo para probar movimientos Cartesianos?
Una vez que el entorno gráfico principal esté corriendo (o el Kinova físico encendido), abre otra terminal y procede con comandos como:
```bash
# Recuerda darle permisos de ejecución primero si falla
chmod +x ~/ros2_ws/src/burger_delivery/scripts/test_kinova_pose.py

# Envía el brazo a X=0.3m Y=0.3m Z=0.3m en el marco de la base (con Roll/Pitch/Yaw por defecto mirando hacia abajo)
python3 ~/ros2_ws/src/burger_delivery/scripts/test_kinova_pose.py --x 0.3 --y 0.3 --z 0.3
```
Si deseas probar rotaciones personalizadas de Euler en el efector final:
```bash
python3 ~/ros2_ws/src/burger_delivery/scripts/test_kinova_pose.py --x 0.3 --y 0.3 --z 0.3 --roll 180 --pitch 0 --yaw 45
```

---

## 2. Poses de Seguridad Preconfiguradas
A veces necesitas "guardar" rápidamente el brazo o desenredarlo para su calibrado en Home; no hace falta inventarse cartesianos matemáticos complejos, para probar un repliegue natural usa articulaciones configuradas pre-programadas:

```bash
# Enviar al Home por defecto
python3 ~/ros2_ws/src/burger_delivery/scripts/test_kinova_pose.py --state Home

# Retraerse/comprimirse para moverse o apagar
python3 ~/ros2_ws/src/burger_delivery/scripts/test_kinova_pose.py --state Retract 

# Estrujar todos los joints a 0 rotación (precaución cerca al techo)
python3 ~/ros2_ws/src/burger_delivery/scripts/test_kinova_pose.py --state Vertical
```

---

## 3. Comprobación y Calibrado de Pinza
Sin necesidad de enviar una trayectoria a los motores largos, puedes confirmar si el Hardware Interface responde abriendo y cerrando:

```bash
# Cerrar pinza (0.8 max close limit Robotiq 85)
python3 ~/ros2_ws/src/burger_delivery/scripts/test_kinova_pose.py --gripper 0.8

# Abrir totalmente pinza (0.0 abierto)
python3 ~/ros2_ws/src/burger_delivery/scripts/test_kinova_pose.py --gripper 0.0
```

*Nota Técnica*: El esqueleto interno usa directamente aceleraciones y velocidades al 5% por seguridad ante colisiones durante testing. Puedes anular esto anexando `--speed 1.0` si estas probando rendimientos agresivos para demos de logísitca del Kinova, o visualizando fantasmas dinámicos (`Ghost`) sobre RViz!
