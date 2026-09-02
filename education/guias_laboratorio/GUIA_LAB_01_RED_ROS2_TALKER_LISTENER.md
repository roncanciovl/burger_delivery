# GUÍA DE LABORATORIO 01: CONFIGURACIÓN Y PUESTA EN OPERACIÓN DE LA RED ROS 2 DISTRIBUIDA (TALKER–LISTENER EN ENTORNO MULTIDISPOSITIVO)

---

| FACULTAD | PROGRAMA | ASIGNATURA | SEMESTRE | CÓDIGO GUÍA | REVISIÓN |
|:---|:---|:---|:---:|:---:|:---:|
| Facultad de Ingeniería | Ingeniería Mecatrónica | ROBOT OPERATING SYSTEM - ROS | VIII – IX | GL-AA-F-1 / LAB-01 | 2 |

**Fecha de emisión:** 29/07/2026

---

## 1. CONTROL DE CAMBIOS

| Descripción del cambio | Justificación del cambio | Fecha de elaboración / actualización |
|---|---|:---:|
| Creación e implementación de la Guía de Laboratorio 1 | Diseño de práctica experimental para configurar redes DDS y comunicación distribuida multi-PC en ROS 2 Jazzy. | 29/07/2026 |
| Normalización de la rúbrica y creación del instrumento de evidencia v1.1 | Alineación con intervalos SGDE, corrección del indicador experimental y comprobación individual post-aplicación sin cambiar el experimento realizado. | 08/08/2026 |

---

## 2. INTRODUCCIÓN

### 2.1. Contexto teórico

En la robótica móvil e industrial moderna, los sistemas rara vez ejecutan todas sus tareas computacionales en un único procesador monolítico. Por el contrario, la arquitectura estándar se basa en robótica distribuida, donde múltiples sensores, actuadores, unidades de procesamiento embarcado —como ordenadores a bordo del robot— y estaciones de control en tierra (*Ground Control Stations*) cooperan compartiendo datos a través de una red de comunicación.

ROS 2 (*Robot Operating System 2*) adopta como capa de transporte de datos el estándar industrial DDS (*Data Distribution Service*). A diferencia de ROS 1, que dependía de un nodo centralizador `roscore`, ROS 2 utiliza un modelo completamente descentralizado de descubrimiento por pares (*peer-to-peer*), fundamentado en tres pilares conceptuales:

1. **Grafo computacional descentralizado:** red dinámica compuesta por nodos procesadores de información que intercambian datos mediante tópicos —canales bus de datos bajo el patrón publicador/suscriptor—.
2. **Dominio de ROS 2 (`ROS_DOMAIN_ID`):** mecanismo de aislamiento lógico sobre la red física. Los nodos que comparten un mismo `ROS_DOMAIN_ID` —entero entre 0 y 101 para esta práctica— pertenecen al mismo segmento computacional y pueden descubrirse automáticamente mediante UDP multicast. Los puertos RTPS calculados a partir del dominio no representan necesariamente todos los sockets: DDS también puede utilizar puertos UDP unicast dinámicos.
3. **Middleware DDS e implementación RMW (*ROS Middleware*):** capa de abstracción que gestiona el descubrimiento, la serialización de mensajes y la Calidad de Servicio (QoS). Mientras que `rmw_fastrtps_cpp` es la implementación por defecto indicada en la guía, para esta práctica se utiliza `rmw_cyclonedds_cpp` en ambos dispositivos por su comportamiento esperado en la red Wi-Fi del laboratorio.

### 2.2. Importancia de la práctica

Esta práctica experimental establece las bases operativas de la robótica distribuida. Aprender a configurar, diagnosticar e interconectar dos o más computadores para que compartan tópicos de ROS 2 de forma transparente es un requisito indispensable para el desarrollo de la Celda de Automatización Colaborativa —proyecto integrador del curso—, donde un robot manipulador de 7 DOF debe coordinar sus movimientos en tiempo real con robots móviles a través de la red.

---

## 3. OBJETIVOS

### 3.1. Objetivo general

Configurar, desplegar y diagnosticar una red robótica distribuida en ROS 2 Jazzy operando sobre el middleware DDS, logrando la transmisión transparente de datos en tiempo real entre un nodo publicador (*Talker*) en un dispositivo y un nodo suscriptor (*Listener*) en otro dispositivo conectado a la misma subred.

### 3.2. Objetivos específicos

1. Establecer la conectividad de red de bajo nivel —capas IP y transporte UDP/ICMP— y configurar adecuadamente el entorno de virtualización —WSL2 en modo *mirrored* o reglas de red equivalentes—.
2. Configurar y verificar las variables de entorno de ROS 2 (`ROS_DOMAIN_ID`, `ROS_AUTOMATIC_DISCOVERY_RANGE`, `RMW_IMPLEMENTATION`) para garantizar el aislamiento de dominio y la optimización del canal Wi-Fi.
3. Desplegar el par de nodos `talker` —Dispositivo 1— y `listener` —Dispositivo 2—, inspeccionando el grafo computacional mediante comandos CLI (`ros2 node list`, `ros2 topic list`, `ros2 topic echo`, `ros2 doctor`).
4. Caracterizar experimentalmente el desempeño de la comunicación midiendo frecuencia de publicación (`ros2 topic hz`), latencia del canal e impacto de las políticas de Calidad de Servicio —QoS Reliable frente a Best Effort—.
5. Aplicar la metodología institucional de depuración por capas —Sintaxis → Red/DDS → Lógica— para identificar y resolver fallas comunes de conectividad en redes robóticas.

---

## 4. DESCRIPCIÓN DE LA PRÁCTICA

### 4.1. Orientación para la persona participante y el asistente técnico

La práctica se desarrolla en parejas de trabajo. Cada grupo seleccionará o recibirá dos dispositivos de cómputo conectados a la red inalámbrica del laboratorio (`ROS2_LAB_NET`) o mediante cable de red Ethernet:

- **Dispositivo 1 —Emisor / Talker—:** actuará como la estación de transmisión de datos de prueba.
- **Dispositivo 2 —Receptor / Listener—:** actuará como el nodo suscriptor encargado de verificar la recepción del mensaje y calcular la frecuencia de llegada.

| Dispositivo 1: emisor | Enlace | Dispositivo 2: receptor |
|:---:|:---:|:---:|
| Nodo: `/talker` | | Nodo: `/listener` |
| Tópico: `/chatter` | | Tópico: `/chatter` |
| RMW: CycloneDDS | **Wi-Fi / LAN** | RMW: CycloneDDS |
| `ROS_DOMAIN_ID=[ID]` | UDP multicast / QoS | `ROS_DOMAIN_ID=[ID]` |

### 4.2. Resultados de aprendizaje evaluables (RAE) y ponderación

La práctica evalúa indicadores del RAE 1 del primer corte, relacionados con redes, arquitectura distribuida, experimentación, comunicación técnica, responsabilidad profesional y trabajo en equipo.

| RAE / Indicador del syllabus | Student Outcome (ABET) | Criterio de evaluación de la práctica | Peso en la nota final | Puntos máximos |
|---|---|---|:---:|:---:|
| RAE 1 - Indicador 2.1 | SO2 —Diseño de Soluciones— | Configuración de arquitectura de software y red DDS (`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, subred y reglas de firewall). | 30% | 1.5 puntos |
| RAE transversal - Indicador 6.4 | SO6 —Experimentación y Análisis— | Diagnóstico experimental por capas: mediciones disponibles, QoS, falla inducida, causa y recuperación. | 30% | 1.5 puntos |
| RAE 1 - Indicador 3.1 | SO3 —Comunicación Efectiva— | Documentación técnica reproducible en Markdown/Git, diagramas del grafo computacional y bitácora técnica de resultados. | 20% | 1.0 punto |
| RAE 1 - Indicador 4.1 | SO4 —Responsabilidad Profesional y Ciberseguridad— | Identificación y mitigación de riesgos de ciberseguridad en DDS —aislamiento estricto de dominio, reglas UFW/Windows Defender—. | 10% | 0.5 puntos |
| RAE 1 - Indicador 5.1 | SO5 —Trabajo en Equipo— | Distribución de roles Talker/Listener, coordinación y comprobación individual de contribuciones. | 10% | 0.5 puntos |
| **TOTAL** | | | **100%** | **5.0 puntos** |

### 4.3. Rúbrica detallada por niveles de desempeño

#### SO2: Diseño de Red DDS y Entorno ROS 2 —peso: 30%—

| Nivel de desempeño | Descriptor |
|:---:|---|
| N1 — 0–149 | No demuestra comunicación distribuida funcional o no presenta evidencia obligatoria verificable. |
| N2 — 150–299 | Obtiene comunicación parcial o intermitente; faltan variables, verificaciones o pasos reproducibles. |
| N3 — 300–399 | Establece la comunicación Talker–Listener entre dispositivos y verifica nodos, tópico y variables principales. |
| N4 — 400–474 | Configura correctamente subred, dominio, CycloneDDS e introspección remota; la comunicación es estable y reproducible. |
| N5 — 475–500 | Además de N4, automatiza o documenta inequívocamente la configuración en ambos dispositivos y justifica RMW, dominio y red. |

#### SO6: Experimentación y Análisis de Datos —peso: 30%—

| Nivel de desempeño | Descriptor |
|:---:|---|
| N1 — 0–149 | Afirma que la red funciona o falla sin datos ni protocolo verificable, o no presenta evidencia obligatoria. |
| N2 — 150–299 | Presenta mediciones o diagnóstico parciales, sin causa demostrada o sin comprobar recuperación. |
| N3 — 300–399 | Registra frecuencia, ejecuta una falla inducida y documenta síntoma, corrección y verificación posterior. |
| N4 — 400–474 | Relaciona frecuencia, latencia/pérdida o QoS con síntomas; aísla la causa por capas y demuestra recuperación. |
| N5 — 475–500 | Además de N4, compara cuantitativamente condiciones, explica variaciones y delimita conclusiones con base en los datos. |

#### SO3: Documentación Técnica Reproducible —peso: 20%—

| Nivel de desempeño | Descriptor |
|:---:|---|
| N1 — 0–149 | Documento fragmentario, no atribuible o sin evidencia funcional verificable. |
| N2 — 150–299 | Contiene capturas o resultados aislados; faltan datos, secuencia, interpretación o trazabilidad. |
| N3 — 300–399 | Incluye comandos esenciales, tabla de resultados, análisis y evidencia suficiente para verificar la práctica. |
| N4 — 400–474 | Documento ordenado y atribuible con arquitectura, comandos, resultados, incidencias y conclusiones casi completamente reproducibles. |
| N5 — 475–500 | Además de N4, integra diagramas claros, scripts o registros estructurados y permite repetir el procedimiento sin aclaraciones externas. |

#### SO4: Ciberseguridad y Aislamiento de Red —peso: 10%—

| Nivel de desempeño | Descriptor |
|:---:|---|
| N1 — 0–149 | Interfiere otros grupos, conserva una regla insegura o no presenta evidencia del control aplicado. |
| N2 — 150–299 | Reconoce el riesgo, pero deja verificaciones incompletas o una restauración ambigua. |
| N3 — 300–399 | Usa un dominio asignado, evita interferencias y verifica que cualquier regla temporal de firewall quede restaurada. |
| N4 — 400–474 | Justifica dominio, permisos de red y restauración segura del firewall; documenta consecuencias de una configuración incorrecta. |
| N5 — 475–500 | Además de N4, analiza riesgos residuales de DDS sin cifrado y propone una mitigación viable para el entorno institucional. |

#### SO5: Trabajo en Equipo y Coordinación —peso: 10%—

| Nivel de desempeño | Descriptor |
|:---:|---|
| N1 — 0–149 | El trabajo se concentra en una persona o no existe evidencia verificable de participación. |
| N2 — 150–299 | La distribución de tareas existe, pero la coordinación o la atribución individual es incompleta. |
| N3 — 300–399 | Identifica roles Talker/Listener, coordina la ejecución y demuestra una contribución individual verificable. |
| N4 — 400–474 | Define y alterna responsabilidades cuando es necesario; el Anexo A demuestra comprensión y contribución individual. |
| N5 — 475–500 | Además de N4, explica decisiones compartidas, resolución conjunta de una falla y contribuciones verificables de ambos roles. |

---

## 5. MATERIALES Y EQUIPOS

La práctica se realiza en parejas. Las cantidades indicadas corresponden a cada grupo.

### 5.1. Equipos del laboratorio

| Descripción | Cantidad | Unidad de medida |
|---|:---:|:---:|
| Router Wi-Fi 6 o switch Gigabit Ethernet con tráfico multicast UDP habilitado | 1 | Unidad por laboratorio |
| Computador con Ubuntu 24.04 LTS y ROS 2 Jazzy | 1 | Unidad por grupo |

### 5.2. Equipos de la persona participante

| Descripción | Cantidad | Unidad de medida |
|---|:---:|:---:|
| Laptop con Ubuntu o Windows 11/WSL2 y ROS 2 Jazzy | 1 | Unidad por grupo |
| Cable de red UTP Cat 6 —opcional, recomendado— | 1 | Unidad por grupo |
| Repositorio `burger_delivery` clonado | 1 | Repositorio por grupo |

---

## 6. SEGURIDAD EN EL LABORATORIO

### 6.1. Normas de seguridad física y operativa

1. Mantener ordenados los cables de red y eléctricos para evitar tropezones en el laboratorio.
2. No consumir alimentos ni bebidas cerca de las estaciones de trabajo y equipos de computación.

### 6.2. Normas de ciberseguridad y aislamiento de red DDS

> [!WARNING]
> **Riesgo de interferencia en DDS:** ROS 2 utiliza paquetes UDP multicast para el descubrimiento automático. Si dos equipos utilizan el mismo `ROS_DOMAIN_ID` en la misma red Wi-Fi, sus nodos se descubrirán e interconectarán, lo que puede generar colisiones de tópicos e interferencia entre grupos.

1. **Aislamiento obligatorio:** cada grupo de laboratorio debe acordar un `ROS_DOMAIN_ID` único asignado por el docente.
2. **Configuración de firewall:** en WSL2, permitir UDP entrante solamente desde la subred ROS asignada y conservar bloqueado cualquier otro origen. No deshabilitar globalmente Windows Defender Firewall ni el firewall de Hyper-V.

---

## 7. PROCEDIMIENTO EXPERIMENTAL

### Fase 1: Verificación de la red física e IP de bajo nivel

1. Conecte ambos dispositivos —Dispositivo 1 y Dispositivo 2— al router Wi-Fi del laboratorio.
2. En el Dispositivo 1, abra una terminal y determine su dirección IP asignada:

   ```bash
   ip addr show | grep inet
   ```

   Anote el resultado: `IP_PC1 = ____________________`.

3. En el Dispositivo 2, determine su dirección IP asignada:

   ```bash
   ip addr show | grep inet
   ```

   Anote el resultado: `IP_PC2 = ____________________`.

4. Ejecute una prueba de conectividad ICMP bidireccional:

   ```bash
   # Desde el Dispositivo 1
   ping -c 4 <IP_PC2>

   # Desde el Dispositivo 2
   ping -c 4 <IP_PC1>
   ```

   Verifique que ambos dispositivos respondan y registre el porcentaje de pérdida de paquetes.

> [!NOTE]
> **Usuarios de Windows con WSL2:** si utiliza WSL2, habilite el modo de red reflejada (*Mirrored Mode*) en `%USERPROFILE%\.wslconfig`:
>
> ```ini
> [wsl2]
> networkingMode=mirrored
> ```
>
> Después ejecute `wsl --shutdown` en PowerShell y abra nuevamente WSL2.

5. Si utiliza WSL2, configure en PowerShell como administrador la excepción distribuida del laboratorio:

   ```powershell
   $wslId = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
   $rosSubnet = '192.168.1.0/24'

   New-NetFirewallHyperVRule `
     -Name 'ROS2-Distributed-LAN-HyperV' `
     -DisplayName 'ROS 2 Distributed LAN WSL' `
     -Direction Inbound -VMCreatorId $wslId `
     -Protocol UDP -RemoteAddresses $rosSubnet -Action Allow

   New-NetFirewallRule `
     -Name 'ROS2-Distributed-LAN-Windows' `
     -DisplayName 'ROS 2 Distributed LAN WSL' `
     -Direction Inbound -Protocol UDP -LocalPort Any `
     -RemoteAddress $rosSubnet -Profile Any -Action Allow
   ```

   Esta excepción permite los puertos UDP calculados y dinámicos de DDS, pero sólo acepta tráfico originado en `192.168.1.0/24`. La entrada general debe permanecer en `Block`. No repita los comandos si las reglas ya existen; verifique primero siguiendo la [guía de configuración de red](../../network_setup/ROS2_NETWORK_CONFIG.md#65-verificación-obligatoria).

### Fase 2: Configuración del middleware DDS y variables de entorno

Para esta práctica se configurará CycloneDDS en ambos dispositivos.

1. Instale la implementación RMW:

   ```bash
   sudo apt update
   sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp
   ```

2. Configure las variables de entorno de ROS 2 en `~/.bashrc` en ambos dispositivos. Reemplace `XX` por el dominio asignado por el docente —por ejemplo, `42`—:

   ```bash
   export ROS_DOMAIN_ID=XX
   export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```

3. Cargue las variables en las terminales activas:

   ```bash
   source ~/.bashrc
   ```

4. Verifique que las variables se hayan aplicado correctamente:

   ```bash
   echo "Domain ID: $ROS_DOMAIN_ID"
   echo "Discovery Range: $ROS_AUTOMATIC_DISCOVERY_RANGE"
   echo "RMW Implementation: $RMW_IMPLEMENTATION"
   ```

### Fase 3: Despliegue distribuido del grafo ROS 2 —Talker y Listener—

#### Paso 3.1: Iniciar el nodo publicador —Talker— en el Dispositivo 1

Ejecute el nodo de demostración:

```bash
ros2 run demo_nodes_cpp talker
```

Resultado esperado:

```text
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
```

#### Paso 3.2: Introspección remota del grafo desde el Dispositivo 2

En el Dispositivo 2, sin iniciar todavía el `listener`, abra una terminal con el mismo entorno ROS 2.

1. Liste los nodos activos:

   ```bash
   ros2 node list
   ```

   Debe aparecer `/talker`.

2. Liste los tópicos activos:

   ```bash
   ros2 topic list
   ```

   Debe aparecer `/chatter`.

3. Inspeccione la información técnica del tópico:

   ```bash
   ros2 topic info /chatter
   ```

   Antes de iniciar el `listener`, verifique un publicador y cero suscriptores.

4. Escuche directamente los datos transmitidos por el Dispositivo 1:

   ```bash
   ros2 topic echo /chatter
   ```

   Deben aparecer los mensajes `Hello World: X` provenientes del Dispositivo 1.

#### Paso 3.3: Iniciar el nodo suscriptor —Listener— en el Dispositivo 2

En una nueva terminal del Dispositivo 2, ejecute:

```bash
ros2 run demo_nodes_cpp listener
```

Resultado esperado:

```text
[INFO] [listener]: I heard: [Hello World: X]
```

### Fase 4: Experimentación y medición del desempeño de red

1. En el Dispositivo 2, mida la tasa de publicación del tópico distribuido:

   ```bash
   ros2 topic hz /chatter
   ```

   Registre la frecuencia media: `Frecuencia = _________ Hz`.

2. En el Dispositivo 1, publique un tópico con confiabilidad Best Effort:

   ```bash
   ros2 topic pub /test_qos std_msgs/msg/String "data: 'QoS Test'" \
     --qos-reliability best_effort
   ```

3. En el Dispositivo 2, intente escuchar el tópico primero con la QoS por defecto y luego con una política compatible:

   ```bash
   ros2 topic echo /test_qos
   ros2 topic echo /test_qos --qos-reliability best_effort
   ```

4. Registre los efectos observados al modificar las políticas QoS y distinga compatibilidad de QoS de conectividad IP o descubrimiento DDS.

### Fase 5: Protocolo de depuración por capas —resolución de problemas—

Cada pareja deberá simular y diagnosticar al menos un escenario de falla introducido intencionalmente:

```text
[ CAPA 1: SINTAXIS / ENTORNO ]  → Variables exportadas, source ~/.bashrc, binarios instalados.
[ CAPA 2: RED Y DDS ]           → Ping ICMP, ROS_DOMAIN_ID coincidente, multicast y firewall.
[ CAPA 3: GRAFO ROS 2 / QoS ]   → ros2 node list, ros2 topic list, compatibilidad QoS.
```

1. **Falla A —Conflicto de dominio—:** cambie el `ROS_DOMAIN_ID` del Dispositivo 2 a un número distinto. Verifique que `ros2 node list` deja de detectar `/talker` y documente el hallazgo.
2. **Falla B —Bloqueo de firewall—:** simule el bloqueo de un puerto en UFW con `sudo ufw deny 7400/udp`. Ejecute `ros2 doctor` para inspeccionar el informe del sistema y restablezca las reglas de red al finalizar.

---

## 8. RESULTADOS DE LA PRÁCTICA

Registre los datos experimentales obtenidos durante la práctica.

| Parámetro / experimento | Dispositivo 1 —Talker— | Dispositivo 2 —Listener— | Observaciones técnicas |
|---|---|---|---|
| Dirección IP | | | Red Wi-Fi / Ethernet |
| Sistema operativo / entorno | | | Ubuntu nativo / WSL2 Mirrored |
| `ROS_DOMAIN_ID` asignado | | | Debe coincidir exactamente |
| RMW Implementation | | | `rmw_cyclonedds_cpp` |
| Estado de ping ICMP | Latencia: _____ ms | Pérdida de paquetes: _____ % | Conectividad de bajo nivel |
| Detección en el grafo (`node list`) | Muestra `/talker`: Sí [ ] No [ ] | Muestra `/talker`: Sí [ ] No [ ] | Descubrimiento DDS por UDP |
| Frecuencia medida (`topic hz`) | Frecuencia nominal: 1.0 Hz | Frecuencia medida: _____ Hz | Estabilidad del tópico `/chatter` |

---

## 9. ANÁLISIS DE RESULTADOS

Desarrolle en su informe los siguientes puntos con base en los datos recolectados:

1. **Mecanismo de descubrimiento DDS:** explique técnicamente cómo el Dispositivo 2 logró descubrir `/talker` sin conocer previamente la dirección IP del Dispositivo 1. ¿Qué función cumple UDP multicast en este proceso?
2. **Impacto de `ROS_DOMAIN_ID`:** explique qué cambia en el descubrimiento y la asignación de puertos cuando dos computadores utilizan el mismo dominio frente a dominios diferentes.
3. **Comparativa RMW:** compare el comportamiento esperado de `rmw_cyclonedds_cpp` y `rmw_fastrtps_cpp` en entornos con pérdida de paquetes o latencia variable, diferenciando la expectativa teórica de los datos realmente obtenidos en la práctica.

---

## 10. CONCLUSIONES

Sintetice los hallazgos principales considerando los siguientes ejes:

1. Cumplimiento del objetivo de comunicación distribuida multidispositivo en ROS 2.
2. Importancia de `ROS_DOMAIN_ID` y `RMW_IMPLEMENTATION` para el aislamiento y la operación del sistema.
3. Utilidad del protocolo de depuración por capas ante problemas de conectividad en robótica colaborativa.

---

## 11. PREGUNTAS PARA LA DISCUSIÓN

1. Si en una planta industrial la red Wi-Fi restringe UDP multicast por políticas de seguridad corporativa, ¿qué mecanismo alternativo de ROS 2/DDS —como Fast DDS Discovery Server— se puede implementar para permitir la comunicación distribuida?
2. ¿Por qué la política QoS Reliable en un tópico de alto ancho de banda —por ejemplo, video de cámara HD— puede degradar el rendimiento general frente a Best Effort?
3. En WSL2 para Windows, ¿cuál es la diferencia crucial entre el modo NAT y el modo Mirrored respecto a la comunicación con nodos ROS 2 externos?

---

## 12. BIBLIOGRAFÍA

1. Quigley, M., Gerkey, B., & Smart, W. D. (2015). *Programming Robots with ROS.* O'Reilly Media.
2. Lentin, J. (2024). *ROS 2 Robotics Developer Guide.* Packt Publishing.
3. Open Robotics. (2026). *ROS 2 Documentation: Jazzy Jalisco — About Domain ID and DDS Configuration.* https://docs.ros.org/en/jazzy/
4. Roncancio, H. (2026). *Syllabus y Guía Técnica de ROBOT OPERATING SYSTEM - ROS.* Universidad Militar Nueva Granada.

---

## 13. ANEXO DE EVALUACIÓN POSTERIOR A LA PRÁCTICA

La rúbrica fue normalizada después de realizar el laboratorio. No se agregan experimentos ni entregables técnicos retroactivos. Para atribuir resultados por estudiante se utiliza el Anexo A del [Instrumento ABET del Laboratorio 01](../evidencias_abet/INSTRUMENTO_ABET_LAB_01_RED_ROS2_DISTRIBUIDA.md). La evidencia común conserva la nota académica; la comprobación individual sustenta la interpretación ABET.

---

## 14. APROBACIÓN DE LA GUÍA DE LABORATORIO

| Elaborado por | Revisado por | Aprobado por |
|:---:|:---:|:---:|
| **Ing. Henry Roncancio**<br>Docente | **Director de Programa**<br>Ingeniería Mecatrónica | **Decano(a)**<br>Facultad de Ingeniería |
