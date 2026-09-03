# diagnostico_microros.ps1
# Script de diagnóstico para depurar problemas de comunicación micro-ROS
# Uso: .\diagnostico_microros.ps1

Write-Host "=====================================" -ForegroundColor Cyan
Write-Host " Diagnóstico de micro-ROS Network" -ForegroundColor Cyan
Write-Host "=====================================" -ForegroundColor Cyan

# Configuración. Se autodetecta para funcionar en cualquier máquina; puedes
# sobrescribirla con variables de entorno sin editar el script:
#   $env:AGENT_IP="192.168.1.100"; .\diagnostico_microros.ps1
# IP de origen de la ruta por defecto: es la que ven los demás equipos. En un
# PC con dos redes (institucional y del router ROS) evita elegir la equivocada.
$IP_LOCAL = (
    Find-NetRoute -RemoteIPAddress '1.1.1.1' -ErrorAction SilentlyContinue |
    Where-Object { $_.IPAddress } |
    Select-Object -First 1 -ExpandProperty IPAddress
)
if (-not $IP_LOCAL) {
    $IP_LOCAL = (
        Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
        Where-Object { $_.InterfaceAlias -notmatch 'Loopback' -and $_.IPAddress -notmatch '^(127\.|169\.254\.)' } |
        Sort-Object -Property SkipAsSource, InterfaceMetric |
        Select-Object -First 1 -ExpandProperty IPAddress
    )
}
$AGENT_IP = if ($env:AGENT_IP) { $env:AGENT_IP } else { $IP_LOCAL }
$AGENT_PORT = if ($env:AGENT_PORT) { [int]$env:AGENT_PORT } else { 8888 }
$ESP32_IPS = if ($env:ESP32_IPS) { $env:ESP32_IPS -split '[,\s]+' } else { @("192.168.1.101", "192.168.1.102") }
$ROS_DOMAIN_ID_ESPERADO = if ($env:ROS_DOMAIN_ID_ESPERADO) { $env:ROS_DOMAIN_ID_ESPERADO } else { "0" }

Write-Host "`n[INFO] Configuración actual:" -ForegroundColor Yellow
Write-Host "  - Agent IP: $AGENT_IP (los ESP32 deben apuntar aquí)"
Write-Host "  - Agent Port: $AGENT_PORT"
Write-Host "  - ESP32s a verificar: $($ESP32_IPS -join ', ')"

# ======================================
# 1. Verificar IP del PC Principal
# ======================================
Write-Host "`n[1/9] Verificando IP del PC Principal..." -ForegroundColor Yellow
# No se asume Wi-Fi: el equipo puede estar cableado a la red ROS.
$wifiAdapter = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Where-Object { $_.IPAddress -eq $IP_LOCAL } | Select-Object -First 1

if ($wifiAdapter -and $wifiAdapter.IPAddress -eq $AGENT_IP) {
    Write-Host "  ✅ IP del agente: $($wifiAdapter.IPAddress) (interfaz $($wifiAdapter.InterfaceAlias))" -ForegroundColor Green
    Write-Host "     Debe coincidir con 'agent_ip' en el firmware de los ESP32." -ForegroundColor Gray
} elseif ($wifiAdapter) {
    Write-Host "  ⚠️  IP actual: $($wifiAdapter.IPAddress)" -ForegroundColor Yellow
    Write-Host "     Esperada: $AGENT_IP" -ForegroundColor Yellow
    Write-Host "     Verifica la reserva DHCP en el router o corrige AGENT_IP" -ForegroundColor Red
} else {
    Write-Host "  ❌ No se encontró adaptador WiFi activo" -ForegroundColor Red
}

# ======================================
# 2. Verificar estado del Firewall
# ======================================
Write-Host "`n[2/9] Verificando estado del Firewall de Windows..." -ForegroundColor Yellow
$firewallProfiles = Get-NetFirewallProfile | Select-Object Name, Enabled

foreach ($profile in $firewallProfiles) {
    if ($profile.Enabled) {
        Write-Host "  ⚠️  $($profile.Name): ACTIVADO (puede bloquear micro-ROS)" -ForegroundColor Yellow
    } else {
        Write-Host "  ✅ $($profile.Name): Desactivado" -ForegroundColor Green
    }
}

# Verificar si existe regla para el puerto 8888
$firewallRule = Get-NetFirewallRule -DisplayName "micro-ROS Agent UDP" -ErrorAction SilentlyContinue
if ($firewallRule) {
    Write-Host "  ✅ Regla de firewall para micro-ROS encontrada" -ForegroundColor Green
} else {
    Write-Host "  ⚠️  No hay regla de firewall específica para micro-ROS" -ForegroundColor Yellow
    Write-Host "     Ejecuta: New-NetFirewallRule -DisplayName 'micro-ROS Agent UDP' -Direction Inbound -Protocol UDP -LocalPort $AGENT_PORT -Action Allow" -ForegroundColor Cyan
}


# ======================================
# 3. Verificar si el puerto 8888 está en uso
# ======================================
Write-Host "`n[3/9] Verificando puerto UDP $AGENT_PORT..." -ForegroundColor Yellow
$udpEndpoint = Get-NetUDPEndpoint | Where-Object LocalPort -eq $AGENT_PORT

if ($udpEndpoint) {
    Write-Host "  ✅ Puerto $AGENT_PORT UDP está en uso por:" -ForegroundColor Green
    foreach ($endpoint in $udpEndpoint) {
        $process = Get-Process -Id $endpoint.OwningProcess -ErrorAction SilentlyContinue
        Write-Host "     - Proceso: $($process.ProcessName) (PID: $($endpoint.OwningProcess))" -ForegroundColor White
        Write-Host "     - Dirección local: $($endpoint.LocalAddress):$($endpoint.LocalPort)" -ForegroundColor White
    }
} else {
    Write-Host "  ❌ Puerto $AGENT_PORT UDP NO está en uso" -ForegroundColor Red
    Write-Host "     El agente micro-ROS no está ejecutándose o no está escuchando correctamente" -ForegroundColor Red
    Write-Host "     Ejecuta: ros2 run micro_ros_agent micro_ros_agent udp4 --port $AGENT_PORT -v6" -ForegroundColor Cyan
}

# ======================================
# 4. Ping a ESP32s
# ======================================
Write-Host "`n[4/9] Verificando conectividad ICMP (ping) a ESP32s..." -ForegroundColor Yellow
foreach ($esp_ip in $ESP32_IPS) {
    $pingResult = Test-Connection -ComputerName $esp_ip -Count 2 -Quiet -ErrorAction SilentlyContinue
    if ($pingResult) {
        Write-Host "  ✅ $esp_ip responde a ping" -ForegroundColor Green
    } else {
        Write-Host "  ❌ $esp_ip NO responde a ping" -ForegroundColor Red
        Write-Host "     Verifica que la ESP32 esté conectada a la red WiFi 'ros2'" -ForegroundColor Red
    }
}
# Verificar que la IP de cada ESP32 está en la misma subred que el agente
$agentSubnet = ($AGENT_IP -split '\.')[0..2] -join '.'
foreach ($esp_ip in $ESP32_IPS) {
    $espSubnet = ($esp_ip -split '\.')[0..2] -join '.'
    if ($espSubnet -eq $agentSubnet) {
        Write-Host "  ✅ $esp_ip está en la subred $agentSubnet.x" -ForegroundColor Green
    } else {
        Write-Host "  ⚠️  $esp_ip está en una subred diferente ($espSubnet.x)" -ForegroundColor Yellow
    }
}

# ======================================
# 5. Verificar variables de entorno ROS
# ======================================
Write-Host "`n[5/9] Verificando variables de entorno ROS 2..." -ForegroundColor Yellow

if ($env:ROS_DOMAIN_ID) {
    if ($env:ROS_DOMAIN_ID -eq $ROS_DOMAIN_ID_ESPERADO) {
        Write-Host "  ✅ ROS_DOMAIN_ID = $env:ROS_DOMAIN_ID (coincide con el dominio del proyecto)" -ForegroundColor Green
    } else {
        Write-Host "  ⚠️  ROS_DOMAIN_ID = $env:ROS_DOMAIN_ID (el proyecto usa $ROS_DOMAIN_ID_ESPERADO)" -ForegroundColor Yellow
        Write-Host "     Todas las máquinas y el agente deben compartir el mismo valor." -ForegroundColor Gray
    }
} else {
    Write-Host "  ✅ ROS_DOMAIN_ID no configurado (usa default 0)" -ForegroundColor Green
}

if ($env:ROS_LOCALHOST_ONLY) {
    if ($env:ROS_LOCALHOST_ONLY -eq "0" -or $env:ROS_LOCALHOST_ONLY -eq "false") {
        Write-Host "  ✅ ROS_LOCALHOST_ONLY = $env:ROS_LOCALHOST_ONLY (permite red)" -ForegroundColor Green
    } else {
        Write-Host "  ❌ ROS_LOCALHOST_ONLY = $env:ROS_LOCALHOST_ONLY (BLOQUEA comunicación de red!)" -ForegroundColor Red
        Write-Host "     Configura: `$env:ROS_LOCALHOST_ONLY=0" -ForegroundColor Cyan
    }
} else {
    Write-Host "  ✅ ROS_LOCALHOST_ONLY no configurado (permite red)" -ForegroundColor Green
}

# ======================================
# 6. Verificar Router/Gateway
# ======================================
Write-Host "`n[6/9] Verificando conectividad al Router/Gateway..." -ForegroundColor Yellow

# Obtener el gateway predeterminado
$gateway = Get-NetRoute -DestinationPrefix "0.0.0.0/0" -ErrorAction SilentlyContinue | 
    Select-Object -First 1 -ExpandProperty NextHop

if ($gateway) {
    Write-Host "  Gateway detectado: $gateway" -ForegroundColor White
    
    # Verificar conectividad al router
    $routerPing = Test-Connection -ComputerName $gateway -Count 2 -Quiet -ErrorAction SilentlyContinue
    if ($routerPing) {
        Write-Host "  ✅ Router responde a ping" -ForegroundColor Green
        
        # Verificar que el gateway esté en la misma subred que el agente
        $agentSubnet = ($AGENT_IP -split '\.')[0..2] -join '.'
        $gatewaySubnet = ($gateway -split '\.')[0..2] -join '.'
        
        if ($agentSubnet -eq $gatewaySubnet) {
            Write-Host "  ✅ Router y agente en la misma subred ($agentSubnet.x)" -ForegroundColor Green
        } else {
            Write-Host "  ⚠️  Router ($gatewaySubnet.x) y agente ($agentSubnet.x) en subredes diferentes" -ForegroundColor Yellow
        }
    } else {
        Write-Host "  ❌ Router NO responde a ping" -ForegroundColor Red
        Write-Host "     Problema de conectividad con el router. Verifica:" -ForegroundColor Red
        Write-Host "     - Cable de red o conexión WiFi" -ForegroundColor Yellow
        Write-Host "     - Configuración del adaptador de red" -ForegroundColor Yellow
    }
    
    Write-Host "`n  📋 Información de Red Detallada:" -ForegroundColor Cyan
    
    # Información del adaptador WiFi
    $wifiInterface = Get-NetAdapter | Where-Object { $_.Status -eq "Up" -and $_.Name -like "*Wi-Fi*" } | Select-Object -First 1
    if ($wifiInterface) {
        Write-Host "     Adaptador WiFi: $($wifiInterface.Name)" -ForegroundColor White
        Write-Host "     MAC Address: $($wifiInterface.MacAddress)" -ForegroundColor White
        
        # Obtener SSID conectado
        try {
            $ssid = (netsh wlan show interfaces | Select-String "SSID" | Select-Object -First 1).ToString().Split(":")[1].Trim()
            Write-Host "     SSID Conectado: $ssid" -ForegroundColor White
            
            if ($ssid -ne "ros2") {
                Write-Host "     ⚠️  ADVERTENCIA: No estás conectado al SSID 'ros2'" -ForegroundColor Yellow
            }
        } catch {
            Write-Host "     No se pudo obtener SSID" -ForegroundColor Yellow
        }
    }
    
    # Información DHCP
    Write-Host "`n  📋 Configuración DHCP:" -ForegroundColor Cyan
    $dhcpInfo = Get-NetIPConfiguration | Where-Object { $_.IPv4DefaultGateway -ne $null } | Select-Object -First 1
    if ($dhcpInfo) {
        Write-Host "     Gateway: $($dhcpInfo.IPv4DefaultGateway.NextHop)" -ForegroundColor White
        Write-Host "     DNS Servers: $($dhcpInfo.DNSServer.ServerAddresses -join ', ')" -ForegroundColor White
        Write-Host "     DHCP Enabled: $($dhcpInfo.NetAdapter.DhcpEnabled)" -ForegroundColor White
    }
    
    # Tabla de rutas relevante
    Write-Host "`n  📋 Rutas de Red:" -ForegroundColor Cyan
    $routes = Get-NetRoute -AddressFamily IPv4 | Where-Object { $_.DestinationPrefix -like "192.168.*" -or $_.DestinationPrefix -eq "0.0.0.0/0" } | 
        Select-Object DestinationPrefix, NextHop, RouteMetric -First 5
    foreach ($route in $routes) {
        Write-Host "     $($route.DestinationPrefix) → $($route.NextHop) (Métrica: $($route.RouteMetric))" -ForegroundColor White
    }
    
    Write-Host "`n  📋 Configuración crítica del router a verificar manualmente:" -ForegroundColor Cyan
    Write-Host "     Accede a: http://$gateway (o http://tplinkwifi.net para TP-Link)" -ForegroundColor White
    Write-Host "     " -ForegroundColor White
    Write-Host "     1. AP Isolation DEBE estar DESACTIVADO" -ForegroundColor Yellow
    Write-Host "        Ruta: Advanced → Wireless → Wireless Settings" -ForegroundColor Gray
    Write-Host "        Busca: 'Enable AP Isolation' → Debe estar DESMARCADO" -ForegroundColor Gray
    Write-Host "     " -ForegroundColor White
    Write-Host "     2. Smart Connect (WiFi 6) - Desactivar si causa problemas" -ForegroundColor Yellow
    Write-Host "        Ruta: Advanced → Wireless → Wireless Settings" -ForegroundColor Gray
    Write-Host "     " -ForegroundColor White
    Write-Host "     3. IP fija reservada para este PC ($AGENT_IP)" -ForegroundColor Yellow
    Write-Host "        MAC: $($wifiInterface.MacAddress)" -ForegroundColor Gray
    Write-Host "        Ruta: Advanced → Network → DHCP Server → Address Reservation" -ForegroundColor Gray
    Write-Host "     " -ForegroundColor White
    $udpClient = New-Object System.Net.Sockets.UdpClient
    $udpClient.Client.ReceiveTimeout = 2000
    $testMessage = [System.Text.Encoding]::ASCII.GetBytes("PING")
    
    # Intentar enviar a localhost primero
    $udpClient.Send($testMessage, $testMessage.Length, "127.0.0.1", $AGENT_PORT) | Out-Null
    Write-Host "  ✅ Envío UDP local exitoso" -ForegroundColor Green
    $udpClient.Close()
} catch {
    Write-Host "  ⚠️  Error al enviar UDP: $($_.Exception.Message)" -ForegroundColor Yellow
}

# ======================================
# 9. Resumen y recomendaciones
# ======================================
Write-Host "`n[9/9] Resumen y recomendaciones:" -ForegroundColor Yellow

$issues = @()

if ($wifiAdapter -and $wifiAdapter.IPAddress -ne $AGENT_IP) {
    $issues += "La IP del PC no coincide con la esperada ($AGENT_IP)"
}

$firewallEnabled = ($firewallProfiles | Where-Object { $_.Name -eq "Private" -and $_.Enabled }).Count -gt 0
if ($firewallEnabled -and -not $firewallRule) {
    $issues += "Firewall activo sin regla específica para micro-ROS"
}

if (-not $udpEndpoint) {
    $issues += "El agente micro-ROS no está escuchando en el puerto $AGENT_PORT"
}

$esp32Down = $ESP32_IPS | Where-Object { -not (Test-Connection -ComputerName $_ -Count 1 -Quiet -ErrorAction SilentlyContinue) }
if ($esp32Down.Count -gt 0) {
    $issues += "ESP32(s) no responden: $($esp32Down -join ', ')"
}

if ($env:ROS_LOCALHOST_ONLY -and $env:ROS_LOCALHOST_ONLY -ne "0" -and $env:ROS_LOCALHOST_ONLY -ne "false") {
    $issues += "ROS_LOCALHOST_ONLY está bloqueando comunicación de red"
}

# Verificar conectividad al router
$gateway = Get-NetRoute -DestinationPrefix "0.0.0.0/0" -ErrorAction SilentlyContinue | 
    Select-Object -First 1 -ExpandProperty NextHop
if ($gateway) {
    $routerPing = Test-Connection -ComputerName $gateway -Count 1 -Quiet -ErrorAction SilentlyContinue
    if (-not $routerPing) {
        $issues += "Router/Gateway ($gateway) no responde"
    }
} else {
    $issues += "No se detectó gateway predeterminado"
}

Write-Host ""
if ($issues.Count -eq 0) {
    Write-Host "  ✅ No se detectaron problemas evidentes" -ForegroundColor Green
    Write-Host "  Si aún tienes problemas de comunicación:" -ForegroundColor Cyan
    Write-Host "    1. Verifica los logs del agente micro-ROS (ejecuta con flag -v6)" -ForegroundColor Cyan
    Write-Host "    2. Revisa los logs seriales de las ESP32s" -ForegroundColor Cyan
    Write-Host "    3. Usa Wireshark para capturar tráfico UDP en puerto $AGENT_PORT" -ForegroundColor Cyan
    Write-Host "    4. Verifica que AP Isolation esté DESACTIVADO en el router" -ForegroundColor Cyan
} else {
    Write-Host "  ❌ Se detectaron los siguientes problemas:" -ForegroundColor Red
    foreach ($issue in $issues) {
        Write-Host "    - $issue" -ForegroundColor Red
    }
    
    Write-Host "`n  📋 Pasos sugeridos de corrección:" -ForegroundColor Yellow
    
    if ($firewallEnabled) {
        Write-Host "    1. Desactivar firewall temporalmente para pruebas:" -ForegroundColor Cyan
        Write-Host "       Set-NetFirewallProfile -Profile Private -Enabled False" -ForegroundColor White
    }
    
    if (-not $udpEndpoint) {
        Write-Host "    2. Iniciar el agente micro-ROS:" -ForegroundColor Cyan
        Write-Host "       ros2 run micro_ros_agent micro_ros_agent udp4 --port $AGENT_PORT -v6" -ForegroundColor White
    }
    
    if ($esp32Down.Count -gt 0) {
        Write-Host "    3. Verificar conexión WiFi de las ESP32s al SSID 'ros2'" -ForegroundColor Cyan
    }
    
    if ($gateway -and -not $routerPing) {
        Write-Host "    4. Verificar router/gateway:" -ForegroundColor Cyan
        Write-Host "       - Reiniciar router" -ForegroundColor White
        Write-Host "       - Verificar cable de red o conexión WiFi" -ForegroundColor White
        Write-Host "       - Revisar configuración DHCP y AP Isolation" -ForegroundColor White
    }
}

Write-Host "`n=====================================" -ForegroundColor Cyan
Write-Host " Diagnóstico completado" -ForegroundColor Cyan
Write-Host "=====================================" -ForegroundColor Cyan
