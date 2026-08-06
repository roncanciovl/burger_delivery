/**
 * app.js
 * Lógica segura del cliente para el Monitor de Red Híbrido ROS 2 & Router.
 * Cumple con estándares de seguridad: usa textContent, DOM createElement y evita innerHTML.
 */

// Estado global de la aplicación
const state = {
  devices: [],
  activeFilter: 'all',
  trafficHistory: {
    labels: [],
    tcp: [],
    udp: [],
    dds: [],
    microros: []
  },
  currentTraffic: null,
  isScanning: false,
  syncSeconds: 0,
  isCalibrated: false,
  benchmark: {
    isActive: false,
    sessionName: 'ensayo_01_kinova',
    scenario: 'Linea_Base_WiFi6',
    elapsedSec: 0,
    samplesCount: 0,
    lastFile: null
  }
};

// ==========================================================================
// Inicialización
// ==========================================================================
document.addEventListener('DOMContentLoaded', () => {
  setupSyncTracker();
  setupLivePolling();
  fetchInitialData();
  setupEventListeners();
  setupBenchmarkModule();
  setupCanvasChart();
  setupTopologyRenderer();
  
  // Renderizado visual inmediato con datos por defecto
  renderDevicesTable();
  renderTopology();
});

// ==========================================================================
// Rastreador de Calibración y Estado de Sincronización
// ==========================================================================
function setupSyncTracker() {
  const banner = document.getElementById('sync-banner');
  const title = document.getElementById('sync-banner-title');
  const subtitle = document.getElementById('sync-banner-subtitle');
  const counter = document.getElementById('sync-counter');
  const badge = document.getElementById('sync-time-badge');

  setInterval(() => {
    state.syncSeconds++;
    if (counter) counter.textContent = state.syncSeconds;

    if (state.syncSeconds >= 4 && !state.isCalibrated) {
      state.isCalibrated = true;
      if (banner) banner.classList.add('synced');
      if (title) title.textContent = '✅ Telemetría en Vivo Sincronizada (1 Hz)';
      if (subtitle) subtitle.textContent = 'Monitoreando tráfico DDS, puertos micro-ROS y latencias del Router AX12 en tiempo real.';
    }

    if (state.isCalibrated && badge) {
      badge.textContent = `Muestras activas: ${state.trafficHistory.labels.length || state.syncSeconds} seg`;
    }
  }, 1000);
}

// ==========================================================================
// Polling Continuo en Vivo (Ultra-Resiliente sin Bloqueo de Sockets)
// ==========================================================================
function setupLivePolling() {
  const sseBadge = document.getElementById('sse-badge');
  const sseText = document.getElementById('sse-status-text');

  if (sseBadge && sseText) {
    sseBadge.style.borderColor = 'rgba(16, 185, 129, 0.4)';
    sseBadge.style.color = '#10b981';
    sseText.textContent = 'Conectado en Vivo (1 Hz)';
  }

  // Polling de tráfico cada 1000ms
  setInterval(fetchTrafficSnapshot, 1000);
  // Polling de benchmark cada 1000ms
  setInterval(fetchBenchmarkStatus, 1000);
  // Polling de dispositivos cada 4000ms
  setInterval(fetchDevicesSnapshot, 4000);
}

async function fetchDevicesSnapshot() {
  try {
    const res = await fetch('/api/devices');
    const data = await res.json();
    if (data.devices && data.devices.length > 0) {
      state.devices = data.devices;
      renderDevicesTable();
      renderTopology();
    }
  } catch (e) {
    console.warn('Polling devices:', e);
  }
}


// ==========================================================================
// Carga Inicial de Datos (Resiliente e Independiente)
// ==========================================================================
async function fetchInitialData() {
  // 1. Estado y configuración de red
  fetch('/api/status')
    .then(r => r.json())
    .then(statusRes => {
      if (statusRes.network) {
        const gwEl = document.getElementById('val-gateway-ip');
        if (gwEl) gwEl.textContent = statusRes.network.gateway_ip || '192.168.1.1';
        const subEl = document.getElementById('subnet-display');
        if (subEl) subEl.textContent = `Subred: ${statusRes.network.subnet}`;
      }
      if (statusRes.environment && statusRes.environment.ROS_DOMAIN_ID !== undefined) {
        const rawDomain = String(statusRes.environment.ROS_DOMAIN_ID);
        const match = rawDomain.match(/\d+/);
        const domainId = match ? match[0] : '0';
        const inputDomain = document.getElementById('input-domain-id');
        if (inputDomain) inputDomain.value = domainId;
        const ddsVal = document.getElementById('val-dds-domains');
        if (ddsVal) ddsVal.textContent = `Configurado local: ${domainId}; esperando RTPS`;
      }
    })
    .catch(err => console.warn('Error cargando /api/status:', err));


  // 2. Dispositivos descubiertos
  fetch('/api/devices')
    .then(r => r.json())
    .then(devicesRes => {
      if (devicesRes.devices) {
        state.devices = devicesRes.devices;
        renderDevicesTable();
        renderTopology();
      }
    })
    .catch(err => console.warn('Error cargando /api/devices:', err));

  // 3. Tráfico inicial
  fetch('/api/traffic')
    .then(r => r.json())
    .then(trafficRes => {
      if (trafficRes.history) {
        state.trafficHistory = trafficRes.history;
      }
      if (trafficRes.current) {
        updateTelemetry(trafficRes.current);
      }
    })
    .catch(err => console.warn('Error cargando /api/traffic:', err));
}


async function fetchTrafficSnapshot() {
  try {
    const res = await fetch('/api/traffic');
    const data = await res.json();
    if (data.current) updateTelemetry(data.current);
    if (data.history) state.trafficHistory = data.history;
  } catch (e) {
    console.error(e);
  }
}

// ==========================================================================
// Actualización de KPIs y Telemetría en Vivo
// ==========================================================================
function updateTelemetry(traffic) {
  if (!traffic) return;
  state.currentTraffic = traffic;

  // KPI 1: Gateway Latencia & Jitter
  const latEl = document.getElementById('val-gateway-lat');
  latEl.textContent = `${traffic.gateway_latency_ms || 0} ms`;
  document.getElementById('val-gateway-jitter').textContent = 
    `Jitter: ${traffic.jitter_ms || 0}ms | Pérdida: ${traffic.packet_loss_percent || 0}%`;

  // KPI 2: Ancho de Banda Total
  document.getElementById('val-total-kbps').textContent = (traffic.total_kbps || 0).toFixed(1);
  const rxKb = ((traffic.bytes_recv_rate || 0) / 1024).toFixed(1);
  const txKb = ((traffic.bytes_sent_rate || 0) / 1024).toFixed(1);
  document.getElementById('val-rate-pkts').textContent = `RX: ${rxKb} KB/s | TX: ${txKb} KB/s`;

  // KPI 3: ROS 2 DDS
  const ddsVal = document.getElementById('val-dds-domains');
  if (traffic.active_dds_domains && traffic.active_dds_domains.length > 0) {
    ddsVal.textContent = `Observados: ${traffic.active_dds_domains.join(', ')}`;
  } else if (traffic.rtps_observer_status === 'error') {
    ddsVal.textContent = 'Observador RTPS no disponible';
  } else if (traffic.configured_domain_observable === false) {
    ddsVal.textContent = `Local: ${traffic.configured_ros_domain_id}; rango UDP efímero no observable`;
  } else {
    ddsVal.textContent = `Sin RTPS observado (local: ${traffic.configured_ros_domain_id ?? 0})`;
  }
  document.getElementById('val-dds-kbps').textContent = `DDS estimado: ${(traffic.dds_kbps ?? 0).toFixed(1)} KB/s | Jitter: ${(traffic.dds_jitter_ms ?? 0).toFixed(1)}ms | Pérdida: ${(traffic.dds_packet_loss_percent ?? 0).toFixed(0)}%`;

  // KPI 4: Micro-ROS
  const urosVal = document.getElementById('val-microros-status');
  const urosDot = document.getElementById('microros-dot');
  if (traffic.microros_agent_active) {
    urosVal.textContent = 'Agente Activo';
    urosVal.style.color = '#10b981';
    urosDot.classList.add('active');
    document.getElementById('val-microros-meta').textContent = `PID: ${traffic.microros_agent_pid || 'ROS2'}`;
  } else {
    urosVal.textContent = 'En Espera';
    urosVal.style.color = '#94a3b8';
    urosDot.classList.remove('active');
    document.getElementById('val-microros-meta').textContent = 'Puerto UDP 8888 no detectado';
  }

  // Actualizar estadísticas de protocolos bajo la gráfica
  document.getElementById('chart-stat-udp').textContent = `${(traffic.udp_kbps || 0).toFixed(1)} KB/s`;
  document.getElementById('chart-stat-dds').textContent = `${(traffic.dds_kbps || 0).toFixed(1)} KB/s`;
  document.getElementById('chart-stat-microros').textContent = `${(traffic.microros_kbps || 0).toFixed(1)} KB/s`;
  document.getElementById('chart-stat-tcp').textContent = `${(traffic.tcp_kbps || 0).toFixed(1)} KB/s`;

  // Actualizar historial local para la gráfica si no está lleno
  if (state.trafficHistory.labels) {
    if (state.trafficHistory.labels.length >= 25) {
      state.trafficHistory.labels.shift();
      state.trafficHistory.tcp.shift();
      state.trafficHistory.udp.shift();
      state.trafficHistory.dds.shift();
      state.trafficHistory.microros.shift();
    }
    state.trafficHistory.labels.push(traffic.timestamp || '');
    state.trafficHistory.tcp.push(traffic.tcp_kbps || 0);
    state.trafficHistory.udp.push(traffic.udp_kbps || 0);
    state.trafficHistory.dds.push(traffic.dds_kbps || 0);
    state.trafficHistory.microros.push(traffic.microros_kbps || 0);
  }

  drawTrafficChart();
  renderSocketsList(traffic.active_sockets || []);
}

// ==========================================================================
// Renderizado Seguro de la Tabla de Dispositivos (DOM API)
// ==========================================================================
function renderDevicesTable() {
  const tbody = document.getElementById('devices-tbody');
  tbody.replaceChildren(); // Seguro: limpia sin innerHTML

  const filtered = state.devices.filter(d => {
    if (state.activeFilter === 'all') return true;
    if (state.activeFilter === 'dds') return d.is_dds_active;
    if (state.activeFilter === 'dom-0') {
      const doms = Array.isArray(d.dds_domains) ? d.dds_domains : [];
      return d.is_dds_active && doms.includes(0);
    }
    if (state.activeFilter === 'dom-42') {
      const doms = Array.isArray(d.dds_domains) ? d.dds_domains : [];
      return d.is_dds_active && doms.includes(42);
    }
    if (state.activeFilter === 'robot') return d.role === 'robot';
    if (state.activeFilter === 'esp32') return d.role === 'esp32';
    return true;
  });

  document.getElementById('devices-count').textContent = `${filtered.length} dispositivos`;

  if (filtered.length === 0) {
    const tr = document.createElement('tr');
    tr.className = 'loading-skeleton-row';
    const td = document.createElement('td');
    td.colSpan = 7;

    const box = document.createElement('div');
    box.className = 'table-loader-box';

    const spinner = document.createElement('span');
    spinner.className = 'sync-spinner';

    const msg = document.createElement('span');
    msg.textContent = state.syncSeconds < 4 
      ? '📡 Sondeando subred y descubriendo nodos ROS 2 DDS (tiempo estimado: ~3 seg)...'
      : 'No se encontraron dispositivos con el filtro seleccionado.';

    if (state.syncSeconds < 4) {
      box.appendChild(spinner);
    }
    box.appendChild(msg);
    td.appendChild(box);
    tr.appendChild(td);
    tbody.appendChild(tr);
    return;
  }


  filtered.forEach(d => {
    const tr = document.createElement('tr');
    if (d.is_dds_active) {
      tr.className = 'tr-dds-active';
    }

    // 1. Rol y Nombre
    const tdRole = document.createElement('td');
    const badge = document.createElement('span');
    badge.className = `device-role-badge role-${d.role}${d.is_dds_active ? ' dds-role-highlight' : ''}`;
    badge.textContent = (d.is_dds_active && !d.label?.startsWith('⚡') ? '⚡ ' : '') + (d.label || d.role);
    tdRole.appendChild(badge);
    tr.appendChild(tdRole);

    // 2. IP
    const tdIp = document.createElement('td');
    tdIp.className = `ip-cell${d.is_dds_active ? ' ip-dds' : ''}`;
    tdIp.textContent = d.ip;
    tr.appendChild(tdIp);

    // 3. MAC
    const tdMac = document.createElement('td');
    tdMac.textContent = d.mac;
    tr.appendChild(tdMac);

    // 4. Fabricante / Hostname
    const tdVendor = document.createElement('td');
    const vText = d.hostname && d.hostname !== 'Desconocido' ? `${d.vendor} (${d.hostname})` : d.vendor;
    tdVendor.textContent = vText;
    tr.appendChild(tdVendor);

    // 5. Indicador Visual de ROS 2 DDS / Micro-ROS con discriminación de Dominio
    const tdDds = document.createElement('td');
    const ddsBadge = document.createElement('span');
    if (d.is_dds_active) {
      const doms = Array.isArray(d.dds_domains) ? d.dds_domains : [];
      const dot = document.createElement('span');
      dot.className = 'status-dot pulsing';

      if (d.role === 'esp32') {
        ddsBadge.className = 'dds-badge badge-domain-esp32';
        dot.style.background = '#10b981';
      } else if (doms.includes(0) && doms.includes(42)) {
        ddsBadge.className = 'dds-badge badge-domain-multi';
        dot.style.background = 'linear-gradient(135deg, #38bdf8, #c084fc)';
      } else if (doms.includes(42)) {
        ddsBadge.className = 'dds-badge badge-domain-42';
        dot.style.background = '#c084fc';
      } else if (doms.includes(0)) {
        ddsBadge.className = 'dds-badge badge-domain-0';
        dot.style.background = '#38bdf8';
      } else {
        ddsBadge.className = 'dds-badge badge-domain-other';
        dot.style.background = '#f59e0b';
      }

      ddsBadge.appendChild(dot);
      const textNode = document.createTextNode(` ⚡ ${d.dds_protocol || 'ROS 2 DDS'}`);
      ddsBadge.appendChild(textNode);
    } else {
      ddsBadge.className = 'dds-badge dds-badge-inactive';
      ddsBadge.textContent = d.dds_protocol || 'ROS 2 no detectado; Domain desconocido';
    }
    tdDds.appendChild(ddsBadge);
    tr.appendChild(tdDds);

    // 6. Latencia
    const tdLat = document.createElement('td');
    tdLat.textContent = `${d.latency_ms} ms`;
    tr.appendChild(tdLat);

    // 7. Estado
    const tdStatus = document.createElement('td');
    const spanStatus = document.createElement('span');
    spanStatus.className = 'online-tag';
    
    const dot = document.createElement('span');
    dot.className = 'status-dot';
    dot.style.background = 'var(--accent-green)';
    
    spanStatus.appendChild(dot);
    const textNode = document.createTextNode(' Online');
    spanStatus.appendChild(textNode);
    tdStatus.appendChild(spanStatus);
    tr.appendChild(tdStatus);

    tbody.appendChild(tr);
  });
}

// ==========================================================================
// Renderizado Seguro de la Lista de Sockets
// ==========================================================================
function renderSocketsList(sockets) {
  const container = document.getElementById('sockets-list');
  container.replaceChildren();

  if (sockets.length === 0) {
    const p = document.createElement('p');
    p.style.color = 'var(--text-dim)';
    p.style.fontSize = '0.85rem';
    p.textContent = 'No se ha observado tráfico RTPS ni servicios Micro-ROS activos.';
    container.appendChild(p);
    return;
  }

  sockets.forEach(s => {
    const item = document.createElement('div');
    item.className = 'socket-item';

    const left = document.createElement('div');
    const portSpan = document.createElement('span');
    portSpan.className = 'socket-port';
    portSpan.textContent = `${s.proto} :${s.port} `;
    
    const roleSpan = document.createElement('span');
    roleSpan.className = 'socket-role';
    roleSpan.textContent = s.role;

    left.appendChild(portSpan);
    left.appendChild(roleSpan);

    const right = document.createElement('span');
    right.className = 'kpi-badge';
    right.textContent = s.status || 'OK';

    item.appendChild(left);
    item.appendChild(right);
    container.appendChild(item);
  });
}

// ==========================================================================
// Renderizado de la Topología Visual (SVG Interactivo con Indicadores DDS)
// ==========================================================================
function setupTopologyRenderer() {
  window.addEventListener('resize', renderTopology);
}

function renderTopology() {
  const svg = document.getElementById('topology-svg');
  if (!svg) return;

  // Limpiar SVG
  while (svg.firstChild) {
    svg.removeChild(svg.firstChild);
  }

  const width = 700;
  const height = 320;
  const routerPos = { x: width / 2, y: 55 };

  // Posiciones de los nodos secundarios en arco (priorizando nodos con DDS)
  const devices = state.devices.filter(d => d.role !== 'router');
  if (state.activeFilter === 'dds') {
    devices.sort((a, b) => (b.is_dds_active ? 1 : 0) - (a.is_dds_active ? 1 : 0));
  }
  
  // Limitar número de nodos visibles en gráfico de topología a los primeros 12 para mantener claridad visual
  const visibleDevices = devices.slice(0, 12);
  const count = visibleDevices.length;

  const ns = 'http://www.w3.org/2000/svg';

  // 1. Dibujar líneas de conexión y partículas DDS
  visibleDevices.forEach((dev, idx) => {
    const angle = (Math.PI / (count + 1)) * (idx + 1);
    const x = width / 2 + Math.cos(angle) * (width * 0.42);
    const y = routerPos.y + Math.sin(angle) * 200;

    const isDds = dev.is_dds_active;

    // Línea de enlace
    const line = document.createElementNS(ns, 'line');
    line.setAttribute('x1', routerPos.x);
    line.setAttribute('y1', routerPos.y + 20);
    line.setAttribute('x2', x);
    line.setAttribute('y2', y);
    line.setAttribute('stroke', isDds ? 'rgba(168, 85, 247, 0.6)' : 'rgba(255, 255, 255, 0.15)');
    line.setAttribute('stroke-width', isDds ? '2.5' : '1.5');
    if (!isDds) line.setAttribute('stroke-dasharray', '4 4');
    svg.appendChild(line);

    // Animación de partículas de tráfico DDS
    const circlePulse = document.createElementNS(ns, 'circle');
    circlePulse.setAttribute('r', isDds ? '5' : '3');
    circlePulse.setAttribute('fill', isDds ? '#c084fc' : getColorByRole(dev.role));
    if (isDds) {
      circlePulse.setAttribute('filter', 'drop-shadow(0 0 6px #a855f7)');
    }
    
    const anim = document.createElementNS(ns, 'animateMotion');
    anim.setAttribute('path', `M ${routerPos.x} ${routerPos.y + 20} L ${x} ${y}`);
    anim.setAttribute('dur', isDds ? '1.0s' : `${1.5 + (idx * 0.3)}s`);
    anim.setAttribute('repeatCount', 'indefinite');
    circlePulse.appendChild(anim);
    svg.appendChild(circlePulse);

    // Nodo del dispositivo con discriminación de dominio
    drawSvgNode(svg, ns, x, y, dev.label || dev.ip, dev.ip, dev.role, getColorByRole(dev.role, isDds, dev.dds_domains), isDds, dev.dds_protocol, dev.dds_domains);
  });

  // 2. Dibujar Router en el centro
  const routerDev = state.devices.find(d => d.role === 'router') || { ip: '--', label: 'Gateway de red activo' };
  drawSvgNode(svg, ns, routerPos.x, routerPos.y, routerDev.label, routerDev.ip, 'router', '#00f2fe', false, '');
}

function getColorByRole(role, isDds = false, doms = []) {
  if (role === 'router') return '#00f2fe';
  if (role === 'esp32') return '#10b981';
  if (role === 'robot') return '#f97316';
  if (isDds) {
    if (doms && doms.includes(42)) return '#c084fc';
    if (doms && doms.includes(0)) return '#38bdf8';
    return '#c084fc';
  }
  return '#94a3b8';
}

function drawSvgNode(svg, ns, x, y, title, subtitle, role, color, isDds = false, ddsProtocol = '', doms = []) {
  const g = document.createElementNS(ns, 'g');
  g.setAttribute('transform', `translate(${x}, ${y})`);

  let nodeColor = color;
  let tagColor = '#c084fc';
  let tagText = '⚡ ROS 2 DDS';

  if (isDds) {
    const protoStr = ddsProtocol || '';
    if (role === 'esp32') {
      nodeColor = '#10b981';
      tagColor = '#10b981';
      tagText = doms && doms.length > 0 ? `⚡ µROS (Dom ${doms.join(', ')})` : '⚡ µROS';
    } else if (doms && doms.includes(0) && doms.includes(42)) {
      nodeColor = '#e879f9';
      tagColor = '#f472b6';
      tagText = '⚡ DOM 0, 42';
    } else if (doms && doms.includes(42)) {
      nodeColor = '#c084fc';
      tagColor = '#c084fc';
      tagText = '⚡ DOM 42';
    } else if (doms && doms.includes(0)) {
      nodeColor = '#38bdf8';
      tagColor = '#38bdf8';
      tagText = '⚡ DOM 0';
    } else if (protoStr.includes('Domain 42')) {
      nodeColor = '#c084fc';
      tagColor = '#c084fc';
      tagText = '⚡ DOM 42';
    } else if (protoStr.includes('Domain 0')) {
      nodeColor = '#38bdf8';
      tagColor = '#38bdf8';
      tagText = '⚡ DOM 0';
    }
  }

  // Anillo exterior con glow especial para ROS 2 DDS
  const outerCircle = document.createElementNS(ns, 'circle');
  outerCircle.setAttribute('r', isDds ? '24' : '20');
  outerCircle.setAttribute('fill', 'rgba(13, 18, 29, 0.9)');
  outerCircle.setAttribute('stroke', isDds ? nodeColor : color);
  outerCircle.setAttribute('stroke-width', isDds ? '3.5' : '2');
  if (isDds) {
    outerCircle.setAttribute('style', `filter: drop-shadow(0 0 8px ${nodeColor}cc);`);
  }
  g.appendChild(outerCircle);

  // Punto central
  const innerCircle = document.createElementNS(ns, 'circle');
  innerCircle.setAttribute('r', '8');
  innerCircle.setAttribute('fill', isDds ? nodeColor : color);
  g.appendChild(innerCircle);

  // Etiqueta ⚡ DDS / Domain si está activo
  if (isDds) {
    const ddsTag = document.createElementNS(ns, 'text');
    ddsTag.setAttribute('y', '-28');
    ddsTag.setAttribute('text-anchor', 'middle');
    ddsTag.setAttribute('fill', tagColor);
    ddsTag.setAttribute('font-size', '10');
    ddsTag.setAttribute('font-weight', '700');
    ddsTag.setAttribute('font-family', 'var(--font-sans)');
    ddsTag.textContent = tagText;
    g.appendChild(ddsTag);
  }

  // Texto título
  const textTitle = document.createElementNS(ns, 'text');
  textTitle.setAttribute('y', '36');
  textTitle.setAttribute('text-anchor', 'middle');
  textTitle.setAttribute('fill', '#f3f4f6');
  textTitle.setAttribute('font-size', '11');
  textTitle.setAttribute('font-weight', '600');
  textTitle.setAttribute('font-family', 'var(--font-sans)');
  textTitle.textContent = title.length > 20 ? title.substring(0, 18) + '...' : title;
  g.appendChild(textTitle);

  // Texto IP
  const textSub = document.createElementNS(ns, 'text');
  textSub.setAttribute('y', '49');
  textSub.setAttribute('text-anchor', 'middle');
  textSub.setAttribute('fill', '#64748b');
  textSub.setAttribute('font-size', '10');
  textSub.setAttribute('font-family', 'var(--font-mono)');
  textSub.textContent = subtitle;
  g.appendChild(textSub);

  svg.appendChild(g);
}


// ==========================================================================
// Gráfica de Tráfico con HTML5 Canvas 2D
// ==========================================================================
let chartAnimationId = null;

function setupCanvasChart() {
  const canvas = document.getElementById('trafficChart');
  if (!canvas) return;

  function resizeCanvas() {
    canvas.width = canvas.parentElement.clientWidth;
    canvas.height = 200;
  }
  resizeCanvas();
  window.addEventListener('resize', resizeCanvas);

  function draw() {
    renderCanvasChart(canvas);
    chartAnimationId = requestAnimationFrame(draw);
  }
  draw();
}

function renderCanvasChart(canvas) {
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;

  ctx.clearRect(0, 0, w, h);

  const hist = state.trafficHistory;
  if (!hist.labels || hist.labels.length < 2) {
    // Dibujar indicador dinámico de calibración
    const pulseRadius = 6 + Math.sin(Date.now() / 250) * 2;
    ctx.beginPath();
    ctx.arc(w / 2, h / 2 - 16, pulseRadius, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(0, 242, 254, 0.6)';
    ctx.fill();

    ctx.fillStyle = '#00f2fe';
    ctx.font = '600 13px Outfit';
    ctx.textAlign = 'center';
    ctx.fillText(`📡 Calibrando analizador de tráfico (muestra ${state.syncSeconds}/30s)...`, w / 2, h / 2 + 10);

    ctx.fillStyle = '#64748b';
    ctx.font = '11px Outfit';
    ctx.fillText('Recolectando paquetes UDP, DDS y Micro-ROS en tiempo real (1 Hz)', w / 2, h / 2 + 28);
    return;
  }


  // Encontrar valor máximo para escala
  let maxVal = 10;
  const series = [hist.udp, hist.dds, hist.microros, hist.tcp];
  series.forEach(arr => {
    if (arr) {
      arr.forEach(v => {
        if (v > maxVal) maxVal = v;
      });
    }
  });
  maxVal = Math.ceil(maxVal * 1.25);

  // Dibujar líneas de cuadrícula
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
  ctx.lineWidth = 1;
  for (let i = 1; i <= 3; i++) {
    const y = (h / 4) * i;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();

    ctx.fillStyle = '#475569';
    ctx.font = '10px JetBrains Mono';
    ctx.textAlign = 'left';
    ctx.fillText(`${Math.round(maxVal - (maxVal / 4) * i)} KB/s`, 8, y - 4);
  }

  // Función para dibujar una serie con gradiente
  function drawLine(data, strokeColor, fillColor) {
    if (!data || data.length === 0) return;
    const step = w / (data.length - 1);

    ctx.beginPath();
    data.forEach((val, i) => {
      const x = i * step;
      const y = h - (val / maxVal) * (h - 20) - 10;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });

    ctx.strokeStyle = strokeColor;
    ctx.lineWidth = 2.5;
    ctx.stroke();

    // Relleno degradado
    ctx.lineTo(w, h);
    ctx.lineTo(0, h);
    ctx.closePath();
    ctx.fillStyle = fillColor;
    ctx.fill();
  }

  // Dibujar series: UDP (azul), DDS (morado), Micro-ROS (naranja), TCP (verde)
  drawLine(hist.udp, '#38bdf8', 'rgba(56, 189, 248, 0.05)');
  drawLine(hist.dds, '#a855f7', 'rgba(168, 85, 247, 0.1)');
  drawLine(hist.microros, '#f97316', 'rgba(249, 115, 22, 0.08)');
  drawLine(hist.tcp, '#10b981', 'rgba(16, 185, 129, 0.05)');
}

// ==========================================================================
// Configuración de Event Listeners y Modales
// ==========================================================================
function setupEventListeners() {
  // Filtros de tabla
  document.querySelectorAll('.filter-btn').forEach(btn => {
    btn.addEventListener('click', (e) => {
      document.querySelectorAll('.filter-btn').forEach(b => b.classList.remove('active'));
      e.target.classList.add('active');
      state.activeFilter = e.target.dataset.filter;
      renderDevicesTable();
      renderTopology();
    });
  });

  // Selector editable de ROS_DOMAIN_ID
  const btnSaveDomain = document.getElementById('btn-save-domain');
  const inputDomain = document.getElementById('input-domain-id');

  async function updateDomainId() {
    if (!inputDomain) return;
    const val = parseInt(inputDomain.value, 10);
    if (isNaN(val) || val < 0 || val > 232) {
      alert('El ROS_DOMAIN_ID debe ser un entero entre 0 y 232');
      return;
    }
    if (btnSaveDomain) {
      btnSaveDomain.disabled = true;
      btnSaveDomain.textContent = 'Guardando...';
    }
    try {
      const res = await fetch('/api/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ ros_domain_id: val })
      });
      const data = await res.json();
      if (data.status === 'success') {
        const ddsVal = document.getElementById('val-dds-domains');
        if (ddsVal) {
          ddsVal.textContent = data.observable === false
            ? `Local: ${val}; rango UDP efímero no observable`
            : `Configurado local: ${val}; esperando RTPS`;
        }
        fetchDevicesSnapshot();
      }
    } catch (err) {
      console.error('Error guardando ROS_DOMAIN_ID:', err);
    } finally {
      if (btnSaveDomain) {
        btnSaveDomain.disabled = false;
        btnSaveDomain.textContent = 'Aplicar';
      }
    }
  }

  if (btnSaveDomain) btnSaveDomain.addEventListener('click', updateDomainId);
  if (inputDomain) {
    inputDomain.addEventListener('keyup', (e) => {
      if (e.key === 'Enter') updateDomainId();
    });
  }


  // Botón Escanear Red
  const btnScan = document.getElementById('btn-scan');
  btnScan.addEventListener('click', async () => {
    if (state.isScanning) return;
    state.isScanning = true;

    const icon = btnScan.querySelector('.btn-icon');
    const text = document.getElementById('scan-btn-text');
    icon.classList.add('spinning');
    text.textContent = 'Escaneando...';

    try {
      const res = await fetch('/api/scan', { method: 'POST' });
      const data = await res.json();
      if (data.devices) {
        state.devices = data.devices;
        renderDevicesTable();
        renderTopology();
      }
    } catch (err) {
      console.error('Error durante el escaneo:', err);
    } finally {
      state.isScanning = false;
      icon.classList.remove('spinning');
      text.textContent = 'Escanear Red';
    }
  });

  // Botón Test Multicast
  const btnMulticast = document.getElementById('btn-test-multicast');
  const modal = document.getElementById('modal-multicast');
  const modalBody = document.getElementById('modal-multicast-body');
  const modalClose = document.getElementById('modal-close');

  btnMulticast.addEventListener('click', async () => {
    btnMulticast.disabled = true;
    modalBody.replaceChildren();
    
    const p = document.createElement('p');
    p.textContent = 'Enviando paquete UDP Multicast a 239.255.0.1:7400...';
    modalBody.appendChild(p);
    modal.classList.remove('hidden');

    try {
      const res = await fetch('/api/test_multicast', { method: 'POST' });
      const data = await res.json();
      
      modalBody.replaceChildren();
      const h4 = document.createElement('h4');
      h4.style.color = data.success ? '#10b981' : '#ef4444';
      h4.style.marginBottom = '8px';
      h4.textContent = data.success ? '✅ Multicast Funcionando' : '❌ Falla en Multicast';

      const desc = document.createElement('p');
      desc.textContent = data.message;

      const lat = document.createElement('p');
      lat.style.fontSize = '0.8rem';
      lat.style.color = 'var(--text-dim)';
      lat.style.marginTop = '6px';
      lat.textContent = `Tiempo de respuesta: ${data.latency_ms} ms`;

      modalBody.appendChild(h4);
      modalBody.appendChild(desc);
      modalBody.appendChild(lat);
    } catch (err) {
      modalBody.replaceChildren();
      const errP = document.createElement('p');
      errP.style.color = '#ef4444';
      errP.textContent = 'Error al ejecutar la prueba de multicast.';
      modalBody.appendChild(errP);
    } finally {
      btnMulticast.disabled = false;
    }
  });

  modalClose.addEventListener('click', () => {
    modal.classList.add('hidden');
  });
}

// ==========================================================================
// Módulo de Benchmark / Experimento de Telemetría QoS & ROS 2
// ==========================================================================
function setupBenchmarkModule() {
  const btnToggle = document.getElementById('btn-benchmark-toggle');
  const btnDownload = document.getElementById('btn-benchmark-download');
  const sessionInput = document.getElementById('benchmark-session-input');
  const scenarioSelect = document.getElementById('benchmark-scenario-select');

  if (!btnToggle) return;

  btnToggle.addEventListener('click', async () => {
    btnToggle.disabled = true;

    if (!state.benchmark.isActive) {
      // Iniciar Grabación
      const sessionName = sessionInput ? sessionInput.value.trim() || 'ensayo_qos' : 'ensayo_qos';
      const scenario = scenarioSelect ? scenarioSelect.value : 'Linea_Base_WiFi6';

      try {
        const res = await fetch('/api/benchmark/start', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ session_name: sessionName, scenario: scenario })
        });
        const data = await res.json();
        if (data.status === 'started') {
          state.benchmark.isActive = true;
          state.benchmark.sessionName = data.session_name;
          state.benchmark.scenario = data.scenario;
          updateBenchmarkUI(true);
        }
      } catch (err) {
        console.error('Error iniciando benchmark:', err);
      } finally {
        btnToggle.disabled = false;
      }
    } else {
      // Detener Grabación
      try {
        const res = await fetch('/api/benchmark/stop', { method: 'POST' });
        const data = await res.json();
        state.benchmark.isActive = false;
        if (data.filename) {
          state.benchmark.lastFile = data.filename;
        }
        updateBenchmarkUI(false, data);
      } catch (err) {
        console.error('Error deteniendo benchmark:', err);
      } finally {
        btnToggle.disabled = false;
      }
    }
  });

  if (btnDownload) {
    btnDownload.addEventListener('click', () => {
      window.location.href = '/api/benchmark/download';
    });
  }
}

async function fetchBenchmarkStatus() {
  try {
    const res = await fetch('/api/benchmark/status');
    const data = await res.json();

    state.benchmark.isActive = !!data.is_active;
    if (data.last_file) state.benchmark.lastFile = data.last_file;

    if (data.is_active) {
      // Actualizar contadores en vivo
      const mins = Math.floor((data.elapsed_sec || 0) / 60).toString().padStart(2, '0');
      const secs = Math.floor((data.elapsed_sec || 0) % 60).toString().padStart(2, '0');
      
      const timeEl = document.getElementById('b-live-time');
      if (timeEl) timeEl.textContent = `${mins}:${secs}`;

      const sampEl = document.getElementById('b-live-samples');
      if (sampEl) sampEl.textContent = `${data.samples_count || 0}`;

      const rttEl = document.getElementById('b-live-rtt');
      if (rttEl) rttEl.textContent = `${data.current_avg_rtt_ms || 0} ms`;

      const jitEl = document.getElementById('b-live-jitter');
      if (jitEl) jitEl.textContent = `${data.current_avg_jitter_ms || 0} ms`;

      const lossEl = document.getElementById('b-live-loss');
      if (lossEl) lossEl.textContent = `${data.current_avg_loss_percent || 0}%`;

      updateBenchmarkUI(true);
    } else {
      if (data.last_file) {
        const lastEl = document.getElementById('b-last-file');
        if (lastEl) lastEl.textContent = data.last_file;
        const btnDl = document.getElementById('btn-benchmark-download');
        if (btnDl) btnDl.disabled = false;
      }
    }
  } catch (err) {
    // Silencioso para evitar saturar consola
  }
}

function updateBenchmarkUI(isRecording, stopResult = null) {
  const btnToggle = document.getElementById('btn-benchmark-toggle');
  const btnText = document.getElementById('benchmark-btn-text');
  const statusBadge = document.getElementById('benchmark-status-badge');
  const statusText = document.getElementById('benchmark-status-text');
  const btnDownload = document.getElementById('btn-benchmark-download');
  const lastFileEl = document.getElementById('b-last-file');
  const sessionInput = document.getElementById('benchmark-session-input');
  const scenarioSelect = document.getElementById('benchmark-scenario-select');

  if (isRecording) {
    if (btnToggle) {
      btnToggle.classList.add('btn-benchmark-stop');
      btnToggle.title = 'Detener grabación y exportar CSV';
    }
    if (btnText) btnText.textContent = '⏹ Detener y Exportar';
    if (statusBadge) statusBadge.classList.add('recording');
    if (statusText) statusText.textContent = '🔴 GRABANDO TELEMETRÍA EN VIVO';
    if (btnDownload) btnDownload.disabled = true;
    if (sessionInput) sessionInput.disabled = true;
    if (scenarioSelect) scenarioSelect.disabled = true;
  } else {
    if (btnToggle) {
      btnToggle.classList.remove('btn-benchmark-stop');
      btnToggle.title = 'Iniciar registro de telemetría';
    }
    if (btnText) btnText.textContent = 'Iniciar Grabación';
    if (statusBadge) statusBadge.classList.remove('recording');
    if (statusText) statusText.textContent = 'Listo para Experimentar';
    if (sessionInput) sessionInput.disabled = false;
    if (scenarioSelect) scenarioSelect.disabled = false;

    if (stopResult && stopResult.filename) {
      if (lastFileEl) lastFileEl.textContent = stopResult.filename;
      if (btnDownload) btnDownload.disabled = false;
    } else if (state.benchmark.lastFile) {
      if (lastFileEl) lastFileEl.textContent = state.benchmark.lastFile;
      if (btnDownload) btnDownload.disabled = false;
    }
  }
}
