# Experimento: efecto del enlace de la estación sobre el ciclo de control del Kinova Gen3

**Fecha:** 2026-09-08 · **Robot:** Kinova Gen3 7 GDL, `192.168.1.10` · **Estación:** WSL2 sobre Windows

---

## 1. Pregunta

Durante la primera prueba de conectividad con el robot real, el `controller_manager`
acumuló 70 desbordamientos de su ciclo de 100 Hz y la API Kortex reportó
`timeout detected: BaseCyclicClient::Refresh` con un bloqueo de 3.0 s. El monitor
registró la pérdida de telemetría correspondiente.

> **¿El cuello de botella es el enlace de red de la estación, o la capa WSL2?**

## 2. Hipótesis

El tramo **inalámbrico** de la estación introduce jitter incompatible con la sesión de
control cíclica en tiempo real de la API Kortex. Si es así, sustituir WiFi por cable debe
eliminar los desbordamientos sin tocar nada más.

Evidencia previa que la motiva: `ping` al robot y al gateway mostraban el **mismo**
patrón de cola larga (máximos de 26 y 118 ms). Como ambos destinos comparten el primer
salto, el factor común tenía que ser el enlace local.

## 3. Diseño

Experimento A/B de una sola variable. Todo lo demás se mantiene constante: misma máquina,
mismo WSL2, mismo ROS 2 Jazzy, mismo `rmw_cyclonedds_cpp`, mismo `ROS_DOMAIN_ID`, mismo
robot, misma sesión de laboratorio, mismo protocolo de 120 s.

| | Rama A | Rama B |
|---|---|---|
| Etiqueta | `wifi_wsl2` | `ethernet_wsl2` |
| Topología | PC —WiFi→ router TP-Link —Ethernet→ Kinova | PC —Ethernet→ router TP-Link —Ethernet→ Kinova |
| Interfaz en WSL | `eth1`, MAC `e0:0a:f6:48:99:ef` | `eth0`, MAC `6c:24:08:88:dc:9f` |
| IP de la estación | `192.168.1.185` | `192.168.1.42` |

Al pasar a cable, el adaptador inalámbrico **desapareció** de la tabla de interfaces, de
modo que no hay ambigüedad sobre la ruta que tomó el tráfico.

### Protocolo (idéntico en ambas ramas)

```bash
./scripts/benchmark_enlace_kinova.sh <etiqueta> 192.168.1.10 120
python3 ./scripts/analizar_enlace.py benchmark_wifi_wsl2 benchmark_ethernet_wsl2
```

1. Registrar las condiciones exactas de la corrida.
2. Medir latencia al robot y al gateway con 500 paquetes, **sin sesión Kortex abierta**.
3. Levantar el driver con `enable_motion:=false` y sin pinza. **El robot no se mueve.**
4. Grabar 120 s de `/joint_states`, `/rosout` y el diagnóstico en MCAP.
5. Analizar la bolsa fuera de línea.

### Por qué se mide sobre la bolsa y no sobre los avisos del `controller_manager`

`controller_manager` imprime sus tiempos **sólo cuando un ciclo se pasa del
presupuesto**. Promediar esas líneas mide únicamente los ciclos malos y sobrestima el
problema — así se produjo una lectura inicial equivocada de "mediana 12.9 ms". La
distribución que aparece abajo se reconstruye a partir de los tiempos de llegada de
**todos** los mensajes grabados, y por eso es insesgada.

---

## 4. Resultados

### Latencia del enlace (500 paquetes, sin sesión Kortex)

| Destino | Rama | min | avg | max | **mdev** |
|---|---|---:|---:|---:|---:|
| Robot | WiFi | 2.023 | 4.759 | 26.822 | 2.914 ms |
| Robot | **Cable** | **0.709** | **1.425** | **6.806** | **0.353 ms** |
| Gateway | WiFi | 2.442 | 5.431 | 38.391 | 4.156 ms |
| Gateway | **Cable** | 1.146 | 2.771 | 27.776 | 3.418 ms |

La desviación media al robot cae **8×**. El gateway mejora menos porque es el propio
router, con su carga de tráfico de la red del laboratorio.

### Ciclo de control (120 s con el driver activo, robot quieto)

| Métrica | WiFi | **Cable** | Factor |
|---|---:|---:|---:|
| Mensajes `/joint_states` | 9723 | 12706 | |
| Frecuencia media | 72.24 Hz | **99.96 Hz** | nominal = 100 |
| Intervalo **p50** | 10.06 ms | 10.00 ms | — |
| Intervalo **p90** | 20.10 ms | **10.30 ms** | 2.0× |
| Intervalo **p99** | 60.12 ms | **10.61 ms** | **5.7×** |
| Intervalo **máximo** | 3251.11 ms | **20.63 ms** | **158×** |
| Intervalos > 20 ms | 1235 (12.7 %) | **2 (0.016 %)** | **790×** |
| Intervalos > 1000 ms | 1 | **0** | |
| Overruns del `controller_manager` | 132 | **6** | **22×** |
| **Pérdidas de enlace** | **2** | **0** | |

---

## 5. Conclusión

**Hipótesis confirmada.** El enlace inalámbrico era la causa dominante.

Lo revelador es que **la mediana era idéntica en las dos ramas** (10.06 frente a
10.00 ms): por WiFi el ciclo se cumplía perfectamente la mitad del tiempo. Lo que
distingue a un enlace utilizable de uno inservible aquí **no es el promedio, es la
cola**. A la API Kortex no le importa que el 87 % de los ciclos llegue a tiempo; le
importa que ninguno se pase. Por eso una lectura basada en `ping` promedio, o en
`ros2 topic hz`, habría dado por bueno un enlace que rompía la sesión de control cada
minuto.

Con cable, el enlace **sostiene el ciclo**: cero pérdidas de telemetría, cero
transiciones a `ERROR`, y el peor intervalo de 120 s (20.63 ms) queda por debajo del
timeout configurado de 1 s con tres órdenes de magnitud de margen.

### Lo que NO demuestra este experimento

- **WSL2 no queda exonerado del todo.** Con cable persisten 6 desbordamientos y 2
  intervalos por encima de 20 ms, y el driver sigue avisando
  `Could not enable FIFO RT scheduling policy: Operation not permitted`: el lazo de
  control no obtiene prioridad de tiempo real. Es un residuo pequeño, ya no bloqueante,
  pero es la explicación candidata si más adelante aparecen fallos de tolerancia durante
  un movimiento real.
- **n = 1 por condición.** No hay repeticiones ni control de la carga de la red del
  laboratorio, que varía con la actividad de los demás equipos.
- **El robot estaba quieto.** Un `FollowJointTrajectory` en ejecución añade carga de
  escritura al ciclo; los márgenes medidos aquí son un límite superior optimista.

### Consecuencias operativas

1. **La estación que ejecuta el driver debe estar por cable.** No es una recomendación de
   rendimiento: por WiFi la sesión de control se rompe. Coincide con la plataforma de
   referencia del proyecto (§3, *"Ethernet hacia el robot"*) y con el `< 5 ms` que espera
   `INSTALACION_KORTEX.md` §5.1.
2. **Las estaciones cliente (`start_driver:=false`) sí pueden ir por WiFi.** Sólo
   consumen telemetría y diagnóstico por DDS; no sostienen el ciclo de 1 kHz de Kortex.
3. **No intentar PA-08 sobre el robot real desde una estación inalámbrica.**

---

## 6. Amenazas a la validez encontradas durante la ejecución

Dos defectos del instrumental contaminaron datos antes de detectarse. Se corrigieron y se
documentan aquí porque cualquiera que repita el experimento los va a encontrar.

| # | Defecto | Efecto sobre los datos | Corrección |
|---|---|---|---|
| 1 | `ros2 bag record` **ignora `SIGINT` dirigido a su PID** cuando no está en el grupo de procesos de una terminal | El grabador de la rama WiFi siguió vivo y grabó **también** la rama Ethernet en la misma bolsa: aparecía un "intervalo máximo" de 526 s y la frecuencia media caía a 29 Hz | El script lanza el grabador con `setsid` y señaliza al **grupo de procesos**, luego verifica que exista `metadata.yaml` y avisa si sobrevive algún grabador |
| 2 | `kill -9` sobre el proceso padre del launch dejaba huérfano al `ros2_control_node` | La sesión Kortex quedaba abierta (riesgo de `Session already in use`) y seguía publicando `/joint_states` después de terminada la corrida | Se señaliza al grupo completo y se **verifica** que el driver reporte `successfully deactivated` antes de recurrir a `SIGKILL` |

Adicionalmente, el analizador ahora **separa sesiones** dentro de una bolsa: un hueco
mayor a 30 s no es una caída del enlace, es la firma de una grabación contaminada. Avisa
y analiza sólo la primera sesión, en lugar de promediar sobre datos de dos experimentos.

También aprendió a leer bolsas con `--compression-mode file` (`SequentialCompressionReader`)
y bolsas sin `metadata.yaml`, abriendo el `.mcap` directamente — que es justo la ventaja
de MCAP frente a SQLite3 ante un cierre abrupto, y le ocurrirá a cualquiera que corte una
grabación con Ctrl+C.

---

## 7. Datos crudos

En [`datos_experimento_enlace/`](datos_experimento_enlace/): condiciones de cada corrida,
salidas de `ping` y los avisos de desbordamiento del `controller_manager`.

Las bolsas MCAP (11 MB y 4.7 MB) **no se versionan**, conforme al requisito de calidad de
mantener fuera del repositorio los logs grandes y las bolsas ROS. Para regenerarlas:

```bash
./scripts/benchmark_enlace_kinova.sh wifi_wsl2     192.168.1.10 120
./scripts/benchmark_enlace_kinova.sh ethernet_wsl2 192.168.1.10 120
python3 ./scripts/analizar_enlace.py benchmark_wifi_wsl2 benchmark_ethernet_wsl2
```

## 8. Próximo paso sugerido

Repetir la rama `ethernet_wsl2` en **Linux nativo** para cuantificar el residuo que aporta
WSL2 (los 6 desbordamientos y la ausencia de scheduling FIFO). Con el enlace ya
descartado como causa dominante, esa comparación pasa a ser de afinamiento, no de
viabilidad.
