# Borrador de paper — Burger-Cell

> **Estado:** borrador de trabajo (TODO.md §1, «Borrador de Paper Científico / Extended
> Abstract»). Escrito en inglés porque las revistas objetivo del plan editorial
> (`docs/research/PLAN_PUBLICACIONES_Y_RELEASES_ZENODO.md`, hito v1.2.0) son IEEE RA-L,
> IEEE T-ASE y Sensors (MDPI).
>
> **Convenciones del borrador:**
> - `[MEDIDO]` cifra tomada de un experimento ya ejecutado y registrado en el repositorio.
> - `[PENDIENTE: …]` resultado que depende de un experimento aún no ejecutado; el branch
>   que trae el instrumental se indica entre paréntesis.
> - `[VERIFICAR]` en las referencias: comprobar datos bibliográficos antes de enviar.
> - Ninguna cifra `[PENDIENTE]` debe salir del borrador sin su fuente de datos.

---

## Title

**Burger-Cell: A Heterogeneous ROS 2 Testbed for VLM-Guided Spatial Manipulation and
Real-Time QoS Analysis**

Alternative (shorter, RA-L style): *Burger-Cell: An Open ROS 2 Testbed Linking Network QoS,
Fiducial Localization and Vision-Language Grounding in a Collaborative Work Cell*

## Abstract (≈ 200 words, target for extended abstract)

Collaborative work cells are increasingly assembled from heterogeneous parts: a manipulator
driven over Ethernet, mobile platforms and microcontrollers on Wi-Fi, and perception services
that call remote vision-language models (VLMs). Their integration is usually evaluated under
an implicit ideal-network assumption. We present Burger-Cell, an open ROS 2 Jazzy testbed built
around a 6-DoF Kinova Gen3 with a Robotiq 2F-85 gripper, differential carts localized with
AprilTags, micro-ROS nodes, and a Gemini Robotics-ER spatial-reasoning node that grounds natural
language object descriptions into 3D targets for MoveIt Task Constructor. The testbed ships with
reproducible instruments for three questions: (i) how the workstation link affects the robot's
control loop, (ii) how injected latency, jitter and loss (tc/netem) degrade joint-trajectory
tracking, and (iii) how accurate and costly VLM-based 3D localization is against a fiducial
ground truth. On the real robot, moving the driver workstation from Wi-Fi to Ethernet reduced the
99th-percentile `/joint_states` inter-arrival time from 60.12 ms to 10.61 ms and removed
telemetry dropouts `[MEDIDO]`. `[PENDIENTE: una frase con el resultado de netem (E2/E3) y otra
con el error 3D y la latencia de Gemini frente a AprilTag.]` Code, data and teaching material are
released under open licenses with a Zenodo DOI.

**Keywords:** ROS 2, DDS, quality of service, vision-language models, fiducial localization,
collaborative robotics, testbed, engineering education.

---

## 1. Introduction

**Problem.** In a real cell the network is part of the control system. A manipulator driver that
needs a 1 kHz cyclic session, a stream of compressed images, and a VLM request that takes seconds
all share the same links. Failures are rarely loud: in our own cell, a driver workstation on
Wi-Fi kept a nominal median cycle (10.06 ms) while its tail produced multi-second gaps and two
telemetry dropouts in 120 s `[MEDIDO]`.

**Gap.** Existing ROS 2 performance studies measure middleware latency on synthetic workloads
[Maruyama 2016; Kronauer 2021], and VLM robotics work reports task success in lab conditions
[Gemini Robotics 2025]. Few open testbeds connect *network condition → control-loop health →
perception accuracy → manipulation outcome* on the same physical cell with shared instruments.

**Contributions.**

1. An open, documented ROS 2 Jazzy testbed (Kinova Gen3 6-DoF, AprilTag-localized carts,
   micro-ROS, VLM perception) organized as reusable packages (`burger_description`,
   `burger_kinova_reference`, `burger_perception`, `burger_control`, `burger_navigation`,
   `burger_telemetry`, `burger_bringup`).
2. Unbiased instruments for control-loop health: full inter-arrival distributions rebuilt from
   recorded bags rather than from controller overrun warnings, which only sample bad cycles.
3. A controlled network-degradation protocol (E1–E3) with tc/netem and trajectory metrics
   (tracking error, jerk, cadence).
4. A benchmark of zero-shot VLM 3D localization against fiducial ground truth on the same
   image, including latency and uplink cost, and a correction to the common ray-scaling mistake
   when deprojecting with a unit-norm ray.
5. Evidence from use as a teaching platform (ABET-aligned labs), including a catalogue of
   *silent failures* found while validating the guides.

## 2. Related Work

- **ROS 2 and DDS performance.** Middleware latency and QoS studies [Maruyama 2016; Kronauer
  2021]; ROS 2 architecture overview [Macenski 2022].
- **Fiducial localization.** AprilTag family and detectors [Olson 2011; Wang 2016]; homography
  on a reference tag to cancel camera motion (our approach for a wrist camera).
- **Task-level manipulation.** MoveIt Task Constructor [Görner 2019].
- **Mobile navigation.** Nav2 [Macenski 2020].
- **VLM grounding for robotics.** Embodied reasoning models that output 2D points and boxes
  [Gemini Robotics 2025]. `[PENDIENTE: 2-3 trabajos comparables de grounding 3D; revisar
  literatura 2024-2026.]`
- **Network emulation.** tc/netem [Hemminger 2005].

## 3. System Architecture

Hardware and software components (Table I). Figure 1: TF tree
`map → table_link → tag_mesa → tag_carritoN → carN_base_link` and
`world → base_link → … → end_effector_link` (see `tf_tree_diagram.svg`).

**Table I — Testbed components** `[completar versiones exactas al congelar v1.2.0]`

| Layer | Component | Notes |
| :--- | :--- | :--- |
| Manipulator | Kinova Gen3, 6 DoF, Robotiq 2F-85, wrist vision module | Driver `ros2_kortex`, `joint_trajectory_controller` at 100 Hz state |
| Mobile | Differential carts with ESP32 (micro-ROS), TurtleBot3 for SLAM | AprilTag 36h11 on the roof |
| Perception | AprilTag localizer (homography on `tag_mesa`); Gemini Robotics-ER node | Registered depth for deprojection |
| Planning | MoveIt 2 + MoveIt Task Constructor; Nav2 | Pick & place of the burger box |
| Network | Wi-Fi 6 router (TP-Link AX12), Ethernet for the driver host | CycloneDDS, shared domain 0 |
| Tooling | Network monitor, rosbag2 (MCAP + zstd), link and trajectory analyzers | All in the repository |

**Operational rule.** Exactly one *host* station runs the robot driver over Ethernet; all
other stations are DDS clients. A guard in the launch file refuses to start a second driver
(the controller accepts it and silently transfers control).

## 4. Methods

### 4.1 Control-loop health (link experiment)

Metric: inter-arrival time Δt of `/joint_states` messages recorded for T = 120 s with the robot
still; percentiles p50/p90/p99, maximum, count of Δt > 20 ms, controller overruns, and Kortex
timeouts. Branches: `wifi_wsl2`, `ethernet_wsl2` `[MEDIDO]`; `ethernet_native`,
`ethernet_native_rt` `[PENDIENTE: branch claude/todo-wsl2-linux-nativo]`.

### 4.2 Network degradation (E1–E3)

Profiles: E1 none; E2 delay 20 ± 8 ms, 4 % loss; E3 delay 60 ± 25 ms, 15 % loss (normal
distribution), applied with netem on (A) a client station toward the host, and (B) the host
toward the robot. Metrics per run: tracking error of the trajectory controller (RMS, p99, max per
joint), jerk estimated by third-order finite differences with a minimum Δt guard, cadence, and
aborted goals. N ≥ 10 runs per profile `[PENDIENTE: branch claude/todo-netem-estres-red]`.

### 4.3 VLM 3D localization vs. fiducial ground truth

For each trial a single image is used for both methods. Ground truth: center of a 36h11 tag on
the box top, via PnP (IPPE-square). VLM: the model returns a point (y, x) normalized to 0–1000;
the pixel is deprojected with the registered depth (median over a 7 × 7 window). The 3D point is

```latex
\mathbf{p} = \frac{z}{r_z}\,\mathbf{r}, \qquad \mathbf{r} = \frac{K^{-1}[u\;v\;1]^\top}{\lVert K^{-1}[u\;v\;1]^\top \rVert}
```

Using z·r instead (a frequent mistake with unit-norm rays) shortens the point by cos θ; at the
image border of a 1920 × 1080 camera with f ≈ 1000 px and z = 1 m this exceeds 0.2 m of error. Metrics: 3D Euclidean
error (mm), pixel error, inference latency (ms), uplink bytes per request, success rate.
`[PENDIENTE: branch claude/todo-benchmark-gemini-apriltag]`

### 4.4 Statistical treatment

Report median and IQR together with mean ± SD (tails matter for control); Mann–Whitney U or
Kruskal–Wallis across profiles; bootstrap 95 % CIs for p99. `[PENDIENTE: fijar antes de medir]`

## 5. Results

### 5.1 Link experiment `[MEDIDO]`

Source: `burger_kinova_reference/docs/EXPERIMENTO_ENLACE_WIFI_VS_ETHERNET.md`.

| Metric (120 s, robot still) | Wi-Fi | Ethernet |
| :--- | ---: | ---: |
| Mean rate | 72.24 Hz | 99.96 Hz |
| Inter-arrival p50 | 10.06 ms | 10.00 ms |
| Inter-arrival p99 | 60.12 ms | 10.61 ms |
| Inter-arrival max | 3251.11 ms | 20.63 ms |
| Intervals > 20 ms | 1235 (12.7 %) | 2 (0.016 %) |
| Controller overruns | 132 | 6 |
| Telemetry dropouts | 2 | 0 |

The medians are identical: what separates a usable link from an unusable one is the tail, which
average-based tools (`ping` mean, `ros2 topic hz`) hide.

### 5.2 Network degradation `[PENDIENTE]`

Figure: tracking error and jerk vs. profile, per placement (A/B).

### 5.3 VLM vs. AprilTag `[PENDIENTE]`

Table: 3D error (mean, p95, RMSE), pixel error, latency, KB per request, success rate.

### 5.4 Measurement pitfalls found `[MEDIDO]`

Source: `TROUBLESHOOTING.md` §4.8. `ros2 topic bw` reported 1.88 MB/s for a raw 1920 × 1080 stream
whose reliable subscriber received ≈ 162 MB/s, because the tool subscribes best-effort and a
single lost fragment drops the image. The network monitor counted loopback traffic as Wi-Fi.
These are reported as a contribution for other testbeds.

## 6. Discussion

- Tail latency, not averages, decides whether a cyclic robot session survives.
- `[PENDIENTE]` Where the VLM sits in the loop: one call per task (seconds), never inside the
  control loop; cost of the uplink compared with the camera stream.
- Educational use: guides validated by execution, not by reading; five classes of silent
  failure (exit code 0 while failing, silent omission, plausible false data, hiding environment,
  guide contradicting the tool).

## 7. Limitations and threats to validity

n = 1 per condition in the link experiment; lab Wi-Fi load uncontrolled; robot still during link
measurements; WSL2 host (`ethernet_native` branch pending); a tag visible to the VLM may bias the
benchmark (variant with a side tag planned).

## 8. Conclusion `[PENDIENTE]`

## Data and code availability

Repository `roncanciovl/burger_delivery`, Zenodo concept DOI 10.5281/zenodo.21809949. Bags are
regenerated with the scripts listed in each experiment document.

## References `[VERIFICAR todos los datos bibliográficos]`

1. S. Macenski, T. Foote, B. Gerkey, C. Lalancette, W. Woodall, "Robot Operating System 2:
   Design, architecture, and uses in the wild," *Science Robotics*, vol. 7, no. 66, 2022.
2. S. Macenski, F. Martín, R. White, J. Ginés Clavero, "The Marathon 2: A Navigation System,"
   *IEEE/RSJ IROS*, 2020.
3. E. Olson, "AprilTag: A robust and flexible visual fiducial system," *IEEE ICRA*, 2011.
4. J. Wang, E. Olson, "AprilTag 2: Efficient and robust fiducial detection," *IEEE/RSJ IROS*,
   2016.
5. M. Görner, R. Haschke, H. Ritter, J. Zhang, "MoveIt! Task Constructor for Task-Level Motion
   Planning," *IEEE ICRA*, 2019.
6. Gemini Robotics Team, "Gemini Robotics: Bringing AI into the Physical World,"
   arXiv:2503.20020, 2025.
7. Y. Maruyama, S. Kato, T. Azumi, "Exploring the performance of ROS2," *EMSOFT*, 2016.
8. T. Kronauer, J. Pohlmann, M. Matthé, T. Smejkal, G. Fettweis, "Latency Analysis of ROS2
   Multi-Node Systems," *IEEE MFI*, 2021.
9. S. Hemminger, "Network Emulation with NetEm," *Linux Conference Australia*, 2005.

---

## Plan de trabajo del manuscrito

| Sección | Depende de | Estado |
| :--- | :--- | :--- |
| 5.1 Enlace | Experimento ya registrado | Listo |
| 4.1 / 5.1 ramas nativas | `claude/todo-wsl2-linux-nativo` | Pendiente de medir |
| 5.2 netem | `claude/todo-netem-estres-red` | Pendiente de medir |
| 5.3 VLM vs AprilTag | `claude/todo-benchmark-gemini-apriltag` | Pendiente de medir |
| 2 Trabajo relacionado | Revisión bibliográfica | Esqueleto |
| Figuras | `analyze_telemetry_benchmark.py`, `analizar_trayectoria.py`, benchmark Gemini | Pendiente |
