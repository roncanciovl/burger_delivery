# Inyección de degradación de red con `tc/netem`

Herramientas para el pendiente de TODO.md §4 «Inyección de Tráfico y Estrés de Red»: emular
los escenarios E2 y E3 de [`EXPERIMENTO_QOS_TELEMETRIA.md`](../../docs/research/EXPERIMENTO_QOS_TELEMETRIA.md)
y medir cómo cambia el seguimiento de trayectorias del Kinova.

| Pieza | Qué hace |
| :--- | :--- |
| [`perfil_netem.sh`](perfil_netem.sh) | Aplica, retira o muestra un perfil (E1, E2, E3 o `custom`) en una interfaz, opcionalmente sólo hacia una IP y en ambos sentidos. Se retira solo a los 600 s |
| [`benchmark_trayectoria_netem.sh`](../../burger_kinova_reference/scripts/benchmark_trayectoria_netem.sh) | Una corrida: aplica el perfil, graba, ejecuta la secuencia segura y retira netem aunque se interrumpa |
| [`analizar_trayectoria.py`](../../burger_kinova_reference/scripts/analizar_trayectoria.py) | Compara corridas: cadencia, error de seguimiento, *jerk* y eventos de `/rosout` |
| `burger_kinova_reference/trajectory_metrics.py` | Las métricas, puras y con pruebas unitarias |

## Perfiles

| Perfil | netem | Escenario del protocolo |
| :--- | :--- | :--- |
| E1 | ninguno | Línea base |
| E2 | `delay 20ms 8ms distribution normal loss 4%` | Carga típica multi-robot |
| E3 | `delay 60ms 25ms distribution normal loss 15%` | Estrés severo |

## Dónde se aplica: dos experimentos distintos

**A. En una estación cliente (recomendado primero).** El driver corre en la anfitriona por
cable y la estación cliente envía las metas por DDS. netem en el cliente, con
`--destino <IP de la anfitriona>`, degrada sólo el tráfico DDS cliente↔anfitriona: metas de
acción, `/joint_states` y diagnóstico. El lazo de control Kortex no se toca, así que el
brazo no puede temblar por esto; lo esperable es que el cliente vea telemetría vieja, que
el monitor pase a WARN/ERROR y que la secuencia se detenga por seguridad.

**B. En la anfitriona hacia el robot (sólo con supervisión).** `--destino 192.168.1.10`
en la interfaz de la anfitriona degrada la sesión en tiempo real de la API Kortex. Es la
pregunta PI-1 en sentido estricto y reproduce lo que el experimento WiFi vs Ethernet midió
sin querer (p99 de 60 ms, temblor). Empieza por `custom --retardo-ms 5 --jitter-ms 2
--perdida-pct 0` y sube de a poco; **no** apliques E3 aquí con movimiento. Parada de
emergencia a mano y el área despejada.

## Protocolo mínimo (experimento A)

```bash
# En la estación cliente, con el driver corriendo en la anfitriona (192.168.1.20 en el ejemplo)
cd burger_kinova_reference/scripts
./benchmark_trayectoria_netem.sh cliente_E1 E1 wlan0 --destino 192.168.1.20 --sin-movimiento
./benchmark_trayectoria_netem.sh cliente_E2 E2 wlan0 --destino 192.168.1.20 --sin-movimiento
./benchmark_trayectoria_netem.sh cliente_E3 E3 wlan0 --destino 192.168.1.20 --sin-movimiento
# Si lo anterior no da sorpresas, repetir E1 y E2 con movimiento (sin --sin-movimiento)
python3 analizar_trayectoria.py trayectoria_cliente_E1 trayectoria_cliente_E2 trayectoria_cliente_E3 \
    --csv resultados_netem.csv
```

Verifica que el perfil quedó aplicado mirando la línea `rtt min/avg/max/mdev` de
`netem.txt`: con E2 el RTT medio debe subir unos 40 ms (20 ms en cada sentido).

## Limitaciones conocidas

- netem necesita el módulo `sch_netem` (y `ifb` para `--ambos-sentidos`). El kernel de WSL2
  puede no traerlos: el script lo detecta y lo dice. En ese caso usa una estación Linux nativa.
- Sin `--ambos-sentidos` sólo se degrada lo que **sale** de la interfaz.
- Estos scripts no se han ejecutado todavía sobre la red del laboratorio.
