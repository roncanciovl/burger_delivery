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
  isScanning: false
};

// ==========================================================================
// Inicialización
// ==========================================================================
document.addEventListener('DOMContentLoaded', () => {
  setupEventSource();
  fetchInitialData();
  setupEventListeners();
  setupCanvasChart();
  setupTopologyRenderer();
});

// ==========================================================================
// Conexión SSE (Server-Sent Events) en Tiempo Real
// ==========================================================================
function setupEventSource() {
  const sseBadge = document.getElementById('sse-badge');
  const sseText = document.getElementById('sse-status-text');

  try {
    const eventSource = new EventSource('/api/events');

    eventSource.onopen = () => {
      sseBadge.style.borderColor = 'rgba(16, 185, 129, 0.4)';
      sseBadge.style.color = '#10b981';
      sseText.textContent = 'Conectado en Vivo (1 Hz)';
    };

    eventSource.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        updateTelemetry(data.traffic);
      } catch (err) {
        console.error('Error parseando SSE payload', err);
      }
    };

    eventSource.onerror = () => {
      sseBadge.style.borderColor = 'rgba(239, 68, 68, 0.4)';
      sseBadge.style.color = '#ef4444';
      sseText.textContent = 'Reconectando...';
    };
  } catch (e) {
    console.warn('SSE no soportado, activando polling');
    setInterval(fetchTrafficSnapshot, 1500);
  }
}

// ==========================================================================
// Carga Inicial de Datos
// ==========================================================================
async function fetchInitialData() {
  try {
    const [statusRes, devicesRes, trafficRes] = await Promise.all([
      fetch('/api/status').then(r => r.json()),
      fetch('/api/devices').then(r => r.json()),
      fetch('/api/traffic').then(r => r.json())
    ]);

    // Actualizar KPIs de entorno
    if (statusRes.network) {
      document.getElementById('val-gateway-ip').textContent = statusRes.network.gateway_ip || '192.168.1.1';
      document.getElementById('subnet-display').textContent = `Subred: ${statusRes.network.subnet}`;
    }

    if (devicesRes.devices) {
      state.devices = devicesRes.devices;
      renderDevicesTable();
      renderTopology();
    }

    if (trafficRes.history) {
      state.trafficHistory = trafficRes.history;
    }
    if (trafficRes.current) {
      updateTelemetry(trafficRes.current);
    }
  } catch (err) {
    console.error('Error en carga inicial:', err);
  }
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
    ddsVal.textContent = `Dominio: ${traffic.active_dds_domains.join(', ')}`;
  } else {
    ddsVal.textContent = 'Sin tráfico activo';
  }
  document.getElementById('val-dds-kbps').textContent = `Tráfico DDS: ${(traffic.dds_kbps || 0).toFixed(1)} KB/s`;

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
    document.getElementById('val-microros-meta').textContent = 'Puerto 8888 escuchando';
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
    if (state.activeFilter === 'robot') return d.role === 'robot';
    if (state.activeFilter === 'esp32') return d.role === 'esp32';
    return true;
  });

  document.getElementById('devices-count').textContent = `${filtered.length} dispositivos`;

  if (filtered.length === 0) {
    const tr = document.createElement('tr');
    const td = document.createElement('td');
    td.colSpan = 6;
    td.style.textAlign = 'center';
    td.style.color = 'var(--text-muted)';
    td.textContent = 'No se encontraron dispositivos con el filtro actual.';
    tr.appendChild(td);
    tbody.appendChild(tr);
    return;
  }

  filtered.forEach(d => {
    const tr = document.createElement('tr');

    // 1. Rol y Nombre
    const tdRole = document.createElement('td');
    const badge = document.createElement('span');
    badge.className = `device-role-badge role-${d.role}`;
    badge.textContent = d.label || d.role;
    tdRole.appendChild(badge);
    tr.appendChild(tdRole);

    // 2. IP
    const tdIp = document.createElement('td');
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

    // 5. Latencia
    const tdLat = document.createElement('td');
    tdLat.textContent = `${d.latency_ms} ms`;
    tr.appendChild(tdLat);

    // 6. Estado
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
    p.textContent = 'No hay sockets DDS o Micro-ROS en escucha actualmente.';
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
// Renderizado de la Topología Visual (SVG Interactivo)
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

  // Posiciones de los nodos secundarios en arco
  const devices = state.devices.filter(d => d.role !== 'router');
  const count = devices.length;

  // Crear elementos SVG mediante DOMParser o createElementNS
  const ns = 'http://www.w3.org/2000/svg';

  // 1. Dibujar líneas de conexión y paquetes
  devices.forEach((dev, idx) => {
    const angle = (Math.PI / (count + 1)) * (idx + 1);
    const x = width / 2 + Math.cos(angle) * (width * 0.42);
    const y = routerPos.y + Math.sin(angle) * 200;

    // Línea de enlace
    const line = document.createElementNS(ns, 'line');
    line.setAttribute('x1', routerPos.x);
    line.setAttribute('y1', routerPos.y + 20);
    line.setAttribute('x2', x);
    line.setAttribute('y2', y);
    line.setAttribute('stroke', 'rgba(255, 255, 255, 0.15)');
    line.setAttribute('stroke-width', '2');
    line.setAttribute('stroke-dasharray', '4 4');
    svg.appendChild(line);

    // Animación de pulso
    const circlePulse = document.createElementNS(ns, 'circle');
    circlePulse.setAttribute('r', '4');
    circlePulse.setAttribute('fill', getColorByRole(dev.role));
    
    const anim = document.createElementNS(ns, 'animateMotion');
    anim.setAttribute('path', `M ${routerPos.x} ${routerPos.y + 20} L ${x} ${y}`);
    anim.setAttribute('dur', `${1.5 + (idx * 0.4)}s`);
    anim.setAttribute('repeatCount', 'indefinite');
    circlePulse.appendChild(anim);
    svg.appendChild(circlePulse);

    // Nodo del dispositivo
    drawSvgNode(svg, ns, x, y, dev.label || dev.ip, dev.ip, dev.role, getColorByRole(dev.role));
  });

  // 2. Dibujar Router en el centro
  const routerDev = state.devices.find(d => d.role === 'router') || { ip: '192.168.1.1', label: 'Router TP-Link AX12' };
  drawSvgNode(svg, ns, routerPos.x, routerPos.y, routerDev.label, routerDev.ip, 'router', '#00f2fe');
}

function getColorByRole(role) {
  switch (role) {
    case 'router': return '#00f2fe';
    case 'host': return '#4facfe';
    case 'robot': return '#f97316';
    case 'esp32': return '#a855f7';
    default: return '#94a3b8';
  }
}

function drawSvgNode(svg, ns, x, y, title, subtitle, role, color) {
  const g = document.createElementNS(ns, 'g');
  g.setAttribute('transform', `translate(${x}, ${y})`);

  // Anillo exterior con glow
  const outerCircle = document.createElementNS(ns, 'circle');
  outerCircle.setAttribute('r', '22');
  outerCircle.setAttribute('fill', 'rgba(13, 18, 29, 0.9)');
  outerCircle.setAttribute('stroke', color);
  outerCircle.setAttribute('stroke-width', '2.5');
  g.appendChild(outerCircle);

  // Punto central
  const innerCircle = document.createElementNS(ns, 'circle');
  innerCircle.setAttribute('r', '8');
  innerCircle.setAttribute('fill', color);
  g.appendChild(innerCircle);

  // Texto título
  const textTitle = document.createElementNS(ns, 'text');
  textTitle.setAttribute('y', '38');
  textTitle.setAttribute('text-anchor', 'middle');
  textTitle.setAttribute('fill', '#f3f4f6');
  textTitle.setAttribute('font-size', '11');
  textTitle.setAttribute('font-weight', '600');
  textTitle.setAttribute('font-family', 'var(--font-sans)');
  textTitle.textContent = title.length > 20 ? title.substring(0, 18) + '...' : title;
  g.appendChild(textTitle);

  // Texto IP
  const textSub = document.createElementNS(ns, 'text');
  textSub.setAttribute('y', '52');
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
    ctx.fillStyle = '#64748b';
    ctx.font = '12px Outfit';
    ctx.textAlign = 'center';
    ctx.fillText('Esperando flujo de paquetes para graficar...', w / 2, h / 2);
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
    });
  });

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
