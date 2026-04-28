# diagnostico_wifi.ps1
# Script de diagnóstico para verificar la calidad y estado de la conexión WiFi en Windows
# Uso: .\diagnostico_wifi.ps1

# Configuración de salida
$OutputEncoding = [System.Text.Encoding]::UTF8

function Write-Header($text) {
    Write-Host "`n===============================================" -ForegroundColor Cyan
    Write-Host "   $text" -ForegroundColor Cyan
    Write-Host "===============================================" -ForegroundColor Cyan
}

function Write-Section($num, $text) {
    Write-Host "`n[$num/6] $text..." -ForegroundColor Yellow
}

Write-Header "Diagnostico de Red WiFi - Burger Delivery (Windows)"

# ======================================
# 1. Verificar Adaptador WiFi
# ======================================
Write-Section 1 "Verificando Adaptador WiFi"

$wifiAdapter = Get-NetAdapter | Where-Object { $_.InterfaceDescription -like "*Wi-Fi*" -or $_.Name -like "*Wi-Fi*" } | Select-Object -First 1

if (-not $wifiAdapter) {
    Write-Host "  [X] No se encontró ningún adaptador WiFi." -ForegroundColor Red
    return
}

Write-Host "  Nombre: $($wifiAdapter.Name)" -ForegroundColor White
Write-Host "  Descripcion: $($wifiAdapter.InterfaceDescription)" -ForegroundColor White

if ($wifiAdapter.Status -eq "Up") {
    Write-Host "  [OK] Estado: ACTIVO" -ForegroundColor Green
} else {
    Write-Host "  [X] Estado: $($wifiAdapter.Status) (Desconectado)" -ForegroundColor Red
    Write-Host "  Intenta activar el WiFi en la barra de tareas de Windows." -ForegroundColor White
}

# ======================================
# 2. Calidad de Señal y SSID
# ======================================
Write-Section 2 "Analizando Calidad de Senal"

try {
    $interfaceInfo = netsh wlan show interfaces | Out-String
    
    $ssidMatch = [regex]::Match($interfaceInfo, 'SSID\s+:\s+([^\r\n]+)')
    $ssid = if ($ssidMatch.Success) { $ssidMatch.Groups[1].Value.Trim() } else { "" }

    $signalMatch = [regex]::Match($interfaceInfo, '(?:Se.*al|Signal)\s+:\s+(\d+)%')
    $signal = if ($signalMatch.Success) { $signalMatch.Groups[1].Value.Trim() } else { "0" }

    $rateMatch = [regex]::Match($interfaceInfo, '(?:Velocidad.*|Receive rate.*)\s+:\s+([^\r\n]+)')
    $rate = if ($rateMatch.Success) { $rateMatch.Groups[1].Value.Trim() } else { "Desconocida" }

    if ($ssid) {
        Write-Host "  SSID Conectado: $ssid" -ForegroundColor White
        Write-Host "  Velocidad: $rate" -ForegroundColor White
        
        $signalNum = [int]$signal
        if ($signalNum -ge 75) {
            Write-Host "  [OK] Senal: $signal% - Excelente" -ForegroundColor Green
        } elseif ($signalNum -ge 50) {
            Write-Host "  [!] Senal: $signal% - Media (Puede afectar latencia)" -ForegroundColor Yellow
        } else {
            Write-Host "  [X] Senal: $signal% - Debil (Riesgo de desconexion)" -ForegroundColor Red
        }

        if ($ssid -ne "ros2") {
            Write-Host "  [i] Nota: No estas en el SSID 'ros2' (recomendado para el robot)" -ForegroundColor White
        }
    } else {
        Write-Host "  [X] No conectado a ninguna red WiFi." -ForegroundColor Red
    }
} catch {
    Write-Host "  [!] Error al obtener detalles del WiFi via netsh: $_" -ForegroundColor Yellow
}

# ======================================
# 3. Escaneo de Redes (Congestión)
# ======================================
Write-Section 3 "Redes detectadas en el entorno"
try {
    netsh wlan show networks | Select-Object -First 10 | ForEach-Object { Write-Host "    $_" -ForegroundColor Gray }
} catch {
    Write-Host "  No se pudieron listar redes cercanas." -ForegroundColor Gray
}

# ======================================
# 4. Pruebas de Latencia y Conectividad
# ======================================
Write-Section 4 "Pruebas de Conectividad y Latencia"

# Gateway
$gateway = Get-NetIPConfiguration | Where-Object { $_.IPv4DefaultGateway -ne $null } | Select-Object -ExpandProperty IPv4DefaultGateway | Select-Object -First 1 -ExpandProperty NextHop

if ($gateway) {
    Write-Host -NoNewline "  Ping al Gateway ($gateway): " -ForegroundColor White
    $pingGW = Test-Connection -ComputerName $gateway -Count 3 -ErrorAction SilentlyContinue
    if ($pingGW) {
        $avgLatency = ($pingGW | Measure-Object -Property ResponseTime -Average).Average
        Write-Host "OK ($([Math]::Round($avgLatency, 2)) ms)" -ForegroundColor Green
        if ($avgLatency -gt 50) {
            Write-Host "  [!] Latencia alta hacia el router (>50ms)." -ForegroundColor Yellow
        }
    } else {
        Write-Host "FALLIDO" -ForegroundColor Red
    }
} else {
    Write-Host "  [X] No se detectó Gateway." -ForegroundColor Red
}

# Internet
Write-Host -NoNewline "  Ping Internet (8.8.8.8): " -ForegroundColor White
if (Test-Connection -ComputerName 8.8.8.8 -Count 1 -Quiet -ErrorAction SilentlyContinue) {
    Write-Host "OK" -ForegroundColor Green
} else {
    Write-Host "SIN ACCESO WAN" -ForegroundColor Red
}

# DNS
Write-Host -NoNewline "  Resolución DNS (google.com): " -ForegroundColor White
try {
    $dns = [System.Net.Dns]::GetHostAddresses("google.com")
    Write-Host "OK" -ForegroundColor Green
} catch {
    Write-Host "FALLIDO" -ForegroundColor Red
}

# ======================================
# 5. Configuración IP y DNS
# ======================================
Write-Section 5 "Configuracion de Red Local"
$ipConfig = Get-NetIPConfiguration | Where-Object { $_.InterfaceAlias -eq $wifiAdapter.Name } | Select-Object -First 1
Write-Host "  IP Local: $($ipConfig.IPv4Address.IPAddress)" -ForegroundColor White
Write-Host "  DNS Servidores: $($ipConfig.DNSServer.ServerAddresses -join ', ')" -ForegroundColor White

# ======================================
# 6. Recomendaciones
# ======================================
Write-Section 6 "Resumen y Recomendaciones"

if ($wifiAdapter.Status -ne "Up") {
    Write-Host "  - ADVERTENCIA: El adaptador WiFi no está activo." -ForegroundColor Red
}

if ($signalNum -and $signalNum -lt 60) {
    Write-Host "  - SENAL DEBIL: Mejora la ubicacion del robot o el router." -ForegroundColor Yellow
}

if ($ssid -and $ssid -ne "ros2") {
    Write-Host "  - RED: Considera usar un router dedicado con el SSID 'ros2'." -ForegroundColor White
}

if ($avgLatency -gt 100) {
    Write-Host "  - LATENCIA CRITICA: La comunicacion ROS2 fallara con latencia > 100ms." -ForegroundColor Red
}

Write-Header "Diagnóstico Finalizado"
