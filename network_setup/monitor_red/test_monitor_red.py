#!/usr/bin/env python3
"""Pruebas de regresión para atribución de ROS_DOMAIN_ID."""

import json
import os
import time
import unittest
import unittest.mock
from unittest.mock import patch

from device_scanner import DeviceScanner
from firewall_status import _resolve_ros_subnet, evaluate_wsl_firewall
from station_listener import StationListener
from traffic_sniffer import TrafficSniffer


class DomainClassificationTests(unittest.TestCase):
    def setUp(self):
        self.scanner = DeviceScanner.__new__(DeviceScanner)
        self.scanner.gateway_ip = "192.168.1.1"
        self.scanner.local_ip = "192.168.1.100"
        self.scanner.local_ips = {"192.168.1.100", "10.0.0.20"}
        self.scanner.target_domain = 42

    def classify(self, ip, domains=None):
        return self.scanner._classify_role(
            ip,
            "a4:bb:6d:00:00:01",
            "workstation",
            "Lenovo PC",
            domains,
        )

    def test_remote_computer_without_rtps_has_unknown_domain(self):
        result = self.classify("10.0.0.30")
        self.assertFalse(result["is_dds_active"])
        self.assertIsNone(result["domain_id"])
        self.assertEqual(result["dds_domains"], [])
        self.assertEqual(result["domain_source"], "unknown")

    def test_remote_computer_uses_only_observed_domains(self):
        result = self.classify("10.0.0.30", [42, 0, 42])
        self.assertTrue(result["is_dds_active"])
        self.assertEqual(result["domain_id"], 0)
        self.assertEqual(result["dds_domains"], [0, 42])
        self.assertEqual(result["domain_source"], "observed")

    @patch.dict(os.environ, {"ROS_DOMAIN_ID": "42"}, clear=False)
    def test_local_configuration_is_not_reported_as_activity(self):
        result = self.classify("192.168.1.100")
        self.assertFalse(result["is_dds_active"])
        self.assertEqual(result["domain_id"], 42)
        self.assertEqual(result["dds_domains"], [])
        self.assertEqual(result["domain_source"], "configured")

    def test_local_interfaces_are_consolidated(self):
        normalized = self.scanner._normalize_domains_map({
            "192.168.1.100": [0],
            "10.0.0.20": [42],
            "10.0.0.30": [7],
        })
        self.assertEqual(normalized["192.168.1.100"], [0, 42])
        self.assertNotIn("10.0.0.20", normalized)
        self.assertEqual(normalized["10.0.0.30"], [7])


class RtpsObservationTests(unittest.TestCase):
    def test_ephemeral_udp_range_is_not_occupied(self):
        domains = TrafficSniffer._get_observable_domains()
        self.assertIn(0, domains)
        self.assertIn(42, domains)
        self.assertFalse(
            TrafficSniffer._domain_block_overlaps_ranges(42, [(32768, 60999)])
        )
        self.assertTrue(
            TrafficSniffer._domain_block_overlaps_ranges(150, [(32768, 60999)])
        )

    def test_rtps_header_validation(self):
        valid = b"RTPS" + bytes([2, 3, 1, 15]) + bytes(range(1, 13))
        zero_guid = b"RTPS" + bytes(16)
        self.assertTrue(TrafficSniffer._is_rtps_packet(valid))
        self.assertFalse(TrafficSniffer._is_rtps_packet(zero_guid))
        self.assertFalse(TrafficSniffer._is_rtps_packet(b"not RTPS"))

    def test_monitor_listener_is_not_counted_as_dds_activity(self):
        sniffer = TrafficSniffer()
        sniffer._candidate_domains = [232]
        sniffer.start_background_collector(interval=10)
        try:
            time.sleep(0.2)
            self.assertTrue(sniffer._rtps_sniffer_thread.is_alive())
            self.assertEqual(sniffer.get_ip_domains_map(), {})
            self.assertEqual(sniffer._scan_active_ports()["active_dds_domains"], [])
        finally:
            sniffer.stop_background_collector()
            sniffer._rtps_sniffer_thread.join(timeout=2)


class FirewallPolicyTests(unittest.TestCase):
    def setUp(self):
        self.valid_status = {
            "hyperv_rule_exists": True,
            "hyperv_enabled": True,
            "hyperv_direction": "Inbound",
            "hyperv_action": "Allow",
            "hyperv_protocol": "UDP",
            "hyperv_local_ports": "Any",
            "hyperv_remote_addresses": "192.168.1.0/255.255.255.0",
            "windows_rule_exists": True,
            "windows_enabled": True,
            "windows_direction": "Inbound",
            "windows_action": "Allow",
            "windows_profile": "Any",
            "windows_protocol": "UDP",
            "windows_local_ports": "Any",
            "windows_remote_addresses": "192.168.1.0/24",
            "default_inbound_action": "Block",
            "default_outbound_action": "Allow",
            "windows_profiles_secure": True,
            "unsafe_ros_rules": "",
        }

    def test_distributed_wsl_policy_is_compliant(self):
        result = evaluate_wsl_firewall(self.valid_status)
        self.assertTrue(result["compliant"])
        self.assertEqual(result["failed_checks"], [])

    def test_remote_any_is_rejected(self):
        self.valid_status["windows_remote_addresses"] = "Any"
        result = evaluate_wsl_firewall(self.valid_status)
        self.assertFalse(result["compliant"])
        self.assertIn("windows_rule", result["failed_checks"])

    def test_default_inbound_allow_is_rejected(self):
        self.valid_status["default_inbound_action"] = "Allow"
        result = evaluate_wsl_firewall(self.valid_status)
        self.assertFalse(result["compliant"])
        self.assertIn("default_inbound_block", result["failed_checks"])

    def test_legacy_rule_open_to_any_network_is_rejected(self):
        self.valid_status["unsafe_ros_rules"] = "ROS2 Legacy Any"
        result = evaluate_wsl_firewall(self.valid_status)
        self.assertFalse(result["compliant"])
        self.assertIn("legacy_rules_scoped", result["failed_checks"])


class RosSubnetResolutionTests(unittest.TestCase):
    """La subred esperada debe poder adaptarse a redes distintas a la del laboratorio."""

    def resolve_with(self, value):
        entorno = dict(os.environ)
        if value is None:
            entorno.pop("ROS_LAN_SUBNET", None)
        else:
            entorno["ROS_LAN_SUBNET"] = value
        with unittest.mock.patch.dict(os.environ, entorno, clear=True):
            return _resolve_ros_subnet()

    def test_lab_subnet_is_the_default(self):
        self.assertEqual(
            self.resolve_with(None), ("192.168.1.0/24", "192.168.1.0/255.255.255.0")
        )

    def test_custom_subnet_is_honoured_in_both_notations(self):
        self.assertEqual(
            self.resolve_with("10.42.0.0/24"), ("10.42.0.0/24", "10.42.0.0/255.255.255.0")
        )

    def test_host_address_is_normalised_to_its_network(self):
        self.assertEqual(self.resolve_with("192.168.5.37/24")[0], "192.168.5.0/24")

    def test_invalid_value_falls_back_to_the_lab_subnet(self):
        self.assertEqual(self.resolve_with("no-es-una-subred")[0], "192.168.1.0/24")


class StationAnnouncementTests(unittest.TestCase):
    """Anuncios de rol de estación: quién tiene ocupado el robot."""

    def setUp(self):
        self.oyente = StationListener(puerto=0, ttl=10.0)

    def _anuncio(self, **campos):
        base = {
            "v": 1,
            "estacion": "PC-LAB-01",
            "estacion_ip": "192.168.1.42",
            "rol": "anfitriona",
            "verificado": "si",
            "evidencia": "sesión TCP establecida con 192.168.1.10:10000",
            "robot_ip": "192.168.1.10",
            "nodo": "kinova_monitor",
            "ts": time.time(),
        }
        base.update(campos)
        return json.dumps(base).encode("utf-8")

    def test_registra_una_anfitriona(self):
        self.oyente._procesar(self._anuncio(), "192.168.1.42")
        anfitriona = self.oyente.anfitriona()
        self.assertIsNotNone(anfitriona)
        self.assertEqual(anfitriona["estacion"], "PC-LAB-01")
        self.assertEqual(anfitriona["ip"], "192.168.1.42")

    def test_la_ip_viene_del_socket_no_del_mensaje(self):
        """Un anuncio no puede declarar una IP distinta a la suya."""
        self.oyente._procesar(self._anuncio(estacion_ip="10.9.9.9"), "192.168.1.42")
        self.assertEqual(self.oyente.anfitriona()["ip"], "192.168.1.42")

    def test_un_cliente_no_es_anfitriona(self):
        self.oyente._procesar(self._anuncio(rol="cliente"), "192.168.1.77")
        self.assertIsNone(self.oyente.anfitriona())
        self.assertEqual(len(self.oyente.estaciones()), 1)

    def test_sin_verificar_no_cuenta_como_anfitriona(self):
        """Declarar el rol no basta: el nodo sólo lo verifica con sesión establecida."""
        self.oyente._procesar(self._anuncio(verificado="no"), "192.168.1.42")
        self.assertIsNone(self.oyente.anfitriona())

    def test_caduca_por_ttl(self):
        """Si la estación se apaga, su anuncio caduca solo: no hace falta despedida."""
        oyente = StationListener(puerto=0, ttl=0.01)
        oyente._procesar(self._anuncio(), "192.168.1.42")
        self.assertIsNotNone(oyente.anfitriona())
        time.sleep(0.05)
        self.assertIsNone(oyente.anfitriona())
        self.assertEqual(oyente.estaciones(), [])

    def test_ignora_datagramas_invalidos(self):
        for basura in (b"no es json", b"[]", b"null", b'{"v": 99}', b"\xff\xfe"):
            self.oyente._procesar(basura, "192.168.1.42")
        self.assertEqual(self.oyente.estaciones(), [])

    def test_trunca_campos_largos(self):
        """Un anuncio hostil no debe poder inflar la interfaz."""
        self.oyente._procesar(self._anuncio(estacion="X" * 5000), "192.168.1.42")
        self.assertLessEqual(len(self.oyente.estaciones()[0]["estacion"]), 64)

    def test_el_ultimo_anuncio_gana(self):
        self.oyente._procesar(self._anuncio(rol="anfitriona"), "192.168.1.42")
        self.oyente._procesar(self._anuncio(rol="cliente"), "192.168.1.42")
        self.assertIsNone(self.oyente.anfitriona())
        self.assertEqual(len(self.oyente.estaciones()), 1)

    def test_consulta_por_ip(self):
        self.oyente._procesar(self._anuncio(), "192.168.1.42")
        self.assertEqual(self.oyente.rol_de("192.168.1.42")["rol"], "anfitriona")
        self.assertIsNone(self.oyente.rol_de("192.168.1.99"))

    def test_todas_las_consultas_devuelven_la_misma_forma(self):
        """
        rol_de, estaciones y anfitriona deben traer los mismos campos.

        Regresión: rol_de omitía edad_s y el servidor reventaba con KeyError al decorar
        la lista de dispositivos.
        """
        self.oyente._procesar(self._anuncio(), "192.168.1.42")
        por_ip = self.oyente.rol_de("192.168.1.42")
        de_lista = self.oyente.estaciones()[0]
        anfitriona = self.oyente.anfitriona()
        self.assertEqual(set(por_ip), set(de_lista))
        self.assertEqual(set(anfitriona), set(de_lista))
        for entrada in (por_ip, de_lista, anfitriona):
            self.assertIn("edad_s", entrada)
            self.assertIsInstance(entrada["edad_s"], float)

    def test_resumen_expone_el_estado(self):
        self.oyente._procesar(self._anuncio(), "192.168.1.42")
        resumen = self.oyente.resumen()
        self.assertTrue(resumen["listener_activo"])
        self.assertEqual(resumen["anuncios_recibidos"], 1)
        self.assertIsNotNone(resumen["anfitriona"])


if __name__ == "__main__":
    unittest.main()
