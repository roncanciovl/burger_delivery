#!/usr/bin/env python3
"""Verificación de solo lectura de la política ROS 2 para WSL/Hyper-V."""

import ipaddress
import json
import os
import shutil
import subprocess
import threading
import time
from typing import Any, Dict, Tuple


DEFAULT_ROS_SUBNET = "192.168.1.0/24"


def _resolve_ros_subnet() -> Tuple[str, str]:
    """Subred ROS esperada en las reglas de firewall.

    Por defecto es la del laboratorio (192.168.1.0/24). Un equipo cuya red ROS
    sea otra la declara con ROS_LAN_SUBNET (por ejemplo 10.42.0.0/24) sin
    editar este archivo. Se devuelven las dos notaciones que usa Windows:
    CIDR y máscara larga.
    """
    raw = (os.environ.get("ROS_LAN_SUBNET") or "").strip() or DEFAULT_ROS_SUBNET
    try:
        network = ipaddress.ip_network(raw, strict=False)
    except ValueError:
        network = ipaddress.ip_network(DEFAULT_ROS_SUBNET)
    return (
        f"{network.network_address}/{network.prefixlen}",
        f"{network.network_address}/{network.netmask}",
    )


ROS_SUBNET, ROS_SUBNET_WINDOWS = _resolve_ros_subnet()
WSL_CREATOR_ID = "{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}"
HYPERV_RULE_NAME = "ROS2-Distributed-LAN-HyperV"
WINDOWS_RULE_NAME = "ROS2-Distributed-LAN-Windows"

_cache_lock = threading.Lock()
_cache_timestamp = 0.0
_cache_value: Dict[str, Any] = {}


def _is_wsl() -> bool:
    try:
        with open("/proc/version", "r", encoding="utf-8") as version_file:
            return "microsoft" in version_file.read().lower()
    except OSError:
        return False


def _contains_expected_subnet(value: Any) -> bool:
    if isinstance(value, list):
        candidates = [str(item).strip() for item in value]
    else:
        candidates = [item.strip() for item in str(value or "").split(",")]
    return ROS_SUBNET in candidates or ROS_SUBNET_WINDOWS in candidates


def _is_any(value: Any) -> bool:
    if isinstance(value, list):
        candidates = value
    else:
        candidates = str(value or "").split(",")
    return any(str(item).strip().lower() == "any" for item in candidates)


def evaluate_wsl_firewall(raw: Dict[str, Any]) -> Dict[str, Any]:
    """Evalúa la salida normalizada de PowerShell contra la política del proyecto."""
    checks = {
        "hyperv_rule": bool(raw.get("hyperv_rule_exists"))
        and bool(raw.get("hyperv_enabled"))
        and str(raw.get("hyperv_direction", "")).lower() == "inbound"
        and str(raw.get("hyperv_action", "")).lower() == "allow"
        and str(raw.get("hyperv_protocol", "")).lower() == "udp"
        and _is_any(raw.get("hyperv_local_ports"))
        and _contains_expected_subnet(raw.get("hyperv_remote_addresses")),
        "windows_rule": bool(raw.get("windows_rule_exists"))
        and bool(raw.get("windows_enabled"))
        and str(raw.get("windows_direction", "")).lower() == "inbound"
        and str(raw.get("windows_action", "")).lower() == "allow"
        and str(raw.get("windows_protocol", "")).lower() == "udp"
        and _is_any(raw.get("windows_local_ports"))
        and _is_any(raw.get("windows_profile"))
        and _contains_expected_subnet(raw.get("windows_remote_addresses")),
        "default_inbound_block": str(raw.get("default_inbound_action", "")).lower()
        == "block",
        "default_outbound_allow": str(raw.get("default_outbound_action", "")).lower()
        == "allow",
        "windows_default_policy": bool(raw.get("windows_profiles_secure")),
        "legacy_rules_scoped": not bool(str(raw.get("unsafe_ros_rules", "")).strip()),
    }
    failed_checks = [name for name, passed in checks.items() if not passed]
    compliant = not failed_checks
    return {
        "platform": "wsl",
        "supported": True,
        "compliant": compliant,
        "expected_subnet": ROS_SUBNET,
        "checks": checks,
        "failed_checks": failed_checks,
        "details": raw,
        "message": (
            "UDP entrante hacia WSL permitido sólo desde la subred ROS; "
            "otras redes permanecen bloqueadas."
            if compliant
            else "La política de firewall ROS 2 requiere revisión."
        ),
    }


def _query_powershell() -> Dict[str, Any]:
    powershell = shutil.which("powershell.exe")
    if not powershell:
        raise RuntimeError("powershell.exe no está disponible desde WSL")

    script = rf"""
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)
$OutputEncoding = [Console]::OutputEncoding
$wslId = '{WSL_CREATOR_ID}'
$hypervRule = Get-NetFirewallHyperVRule -Name '{HYPERV_RULE_NAME}' -ErrorAction SilentlyContinue
$windowsRule = Get-NetFirewallRule -Name '{WINDOWS_RULE_NAME}' -ErrorAction SilentlyContinue
$windowsPort = $windowsRule | Get-NetFirewallPortFilter -ErrorAction SilentlyContinue
$windowsAddress = $windowsRule | Get-NetFirewallAddressFilter -ErrorAction SilentlyContinue
$windowsProfiles = @(Get-NetFirewallProfile -PolicyStore ActiveStore -ErrorAction SilentlyContinue)
$hypervProfile = Get-NetFirewallHyperVProfile -PolicyStore ActiveStore |
  Where-Object {{ $_.Name -eq $wslId }} |
  Select-Object -First 1
$unsafeWindowsRules = @(
  Get-NetFirewallRule -PolicyStore ActiveStore -ErrorAction SilentlyContinue |
    Where-Object {{
      $_.Enabled -eq 'True' -and $_.Direction -eq 'Inbound' -and $_.Action -eq 'Allow' -and
      ("$($_.Name) $($_.DisplayName) $($_.DisplayGroup)" -match '(?i)(^|[^A-Z])(ROS2?|DDS)([^A-Z0-9]|$)')
    }} |
    ForEach-Object {{
      $address = $_ | Get-NetFirewallAddressFilter -ErrorAction SilentlyContinue
      if (@($address.RemoteAddress) -contains 'Any') {{ $_.DisplayName }}
    }}
)
$unsafeHyperVRules = @(
  Get-NetFirewallHyperVRule -PolicyStore ActiveStore -ErrorAction SilentlyContinue |
    Where-Object {{
      $_.Enabled -eq 'True' -and $_.Direction -eq 'Inbound' -and $_.Action -eq 'Allow' -and
      ("$($_.Name) $($_.DisplayName)" -match '(?i)(^|[^A-Z])(ROS2?|DDS)([^A-Z0-9]|$)') -and
      (@($_.RemoteAddresses) -contains 'Any')
    }} |
    ForEach-Object {{ $_.DisplayName }}
)
$unsafeRosRules = @($unsafeWindowsRules) + @($unsafeHyperVRules)
$windowsProfilesSecure = $windowsProfiles.Count -gt 0 -and -not @(
  $windowsProfiles | Where-Object {{
    "$( $_.DefaultInboundAction )" -ne 'Block' -or
    "$( $_.DefaultOutboundAction )" -ne 'Allow'
  }}
).Count

[PSCustomObject]@{{
  hyperv_rule_exists       = [bool]$hypervRule
  hyperv_enabled           = [bool]($hypervRule -and $hypervRule.Enabled)
  hyperv_direction         = "$($hypervRule.Direction)"
  hyperv_action            = "$($hypervRule.Action)"
  hyperv_protocol          = "$($hypervRule.Protocol)"
  hyperv_local_ports       = @($hypervRule.LocalPorts) -join ','
  hyperv_remote_addresses  = @($hypervRule.RemoteAddresses) -join ','
  windows_rule_exists      = [bool]$windowsRule
  windows_enabled          = [bool]($windowsRule -and $windowsRule.Enabled)
  windows_direction        = "$($windowsRule.Direction)"
  windows_action           = "$($windowsRule.Action)"
  windows_profile          = "$($windowsRule.Profile)"
  windows_protocol         = "$($windowsPort.Protocol)"
  windows_local_ports      = @($windowsPort.LocalPort) -join ','
  windows_remote_addresses = @($windowsAddress.RemoteAddress) -join ','
  default_inbound_action   = "$($hypervProfile.DefaultInboundAction)"
  default_outbound_action  = "$($hypervProfile.DefaultOutboundAction)"
  windows_profiles_secure  = [bool]$windowsProfilesSecure
  unsafe_ros_rules         = @($unsafeRosRules) -join ', '
}} | ConvertTo-Json -Compress
"""
    result = subprocess.run(
        [powershell, "-NoProfile", "-NonInteractive", "-Command", script],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=8.0,
        check=False,
    )
    if result.returncode != 0:
        error = result.stderr.strip() or f"PowerShell terminó con código {result.returncode}"
        raise RuntimeError(error)

    output = result.stdout.strip()
    if not output:
        raise RuntimeError("PowerShell no devolvió el estado del firewall")
    return json.loads(output)


def get_firewall_status(force: bool = False, cache_seconds: float = 30.0) -> Dict[str, Any]:
    """Retorna el cumplimiento de firewall sin solicitar privilegios ni modificar reglas."""
    global _cache_timestamp, _cache_value

    if not _is_wsl():
        return {
            "platform": "native_linux",
            "supported": False,
            "compliant": None,
            "expected_subnet": ROS_SUBNET,
            "checks": {},
            "failed_checks": [],
            "details": {},
            "message": "Verificación Windows/Hyper-V no aplicable en Linux nativo.",
        }

    now = time.monotonic()
    with _cache_lock:
        if not force and _cache_value and now - _cache_timestamp < cache_seconds:
            return dict(_cache_value)

    try:
        status = evaluate_wsl_firewall(_query_powershell())
    except (
        OSError,
        RuntimeError,
        UnicodeError,
        subprocess.SubprocessError,
        json.JSONDecodeError,
    ) as error:
        status = {
            "platform": "wsl",
            "supported": True,
            "compliant": False,
            "expected_subnet": ROS_SUBNET,
            "checks": {},
            "failed_checks": ["query_failed"],
            "details": {},
            "message": f"No fue posible verificar el firewall: {error}",
        }

    status["checked_at"] = time.strftime("%H:%M:%S")
    with _cache_lock:
        _cache_value = dict(status)
        _cache_timestamp = now
    return status
