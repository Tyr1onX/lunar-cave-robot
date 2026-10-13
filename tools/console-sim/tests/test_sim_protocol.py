import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim_protocol import (HOST, PORT, DOMAIN, COMMAND, ODOM, MAP, SCAN,
                          validate_command, require_isolation)


class ProtocolTests(unittest.TestCase):
    def test_fixed_loopback_and_topics(self):
        self.assertEqual((HOST, PORT, DOMAIN), ("127.0.0.1", 8766, "217"))
        self.assertEqual((COMMAND, ODOM, MAP, SCAN),
                         ("/console_sim/cmd_vel", "/console_sim/odom",
                          "/console_sim/map", "/console_sim/scan"))
        self.assertNotEqual(COMMAND, "/cmd_vel")

    def test_environment_fails_closed(self):
        for domain, localhost in ((None, None), ("0", "1"), ("217", "0"), ("203", "1")):
            with self.subTest(domain=domain, localhost=localhost):
                env = {}
                if domain is not None:
                    env["ROS_DOMAIN_ID"] = domain
                if localhost is not None:
                    env["ROS_LOCALHOST_ONLY"] = localhost
                with patch.dict(os.environ, env, clear=True):
                    with self.assertRaises(SystemExit):
                        require_isolation()
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "217", "ROS_LOCALHOST_ONLY": "1"}, clear=True):
            require_isolation()

    def test_whitelisted_bounded_finite_input(self):
        token = "a" * 32
        self.assertEqual(validate_command({"token":token,"seq":1,"forward":1,"turn":-1}),
                         (token, 1, 1.0, -1.0))
        for bad in (
            {"token":token,"seq":1,"forward":2,"turn":0},
            {"token":token,"seq":1,"forward":float("nan"),"turn":0},
            {"token":token,"seq":1,"forward":float("inf"),"turn":0},
            {"token":token,"seq":0,"forward":0,"turn":0},
            {"token":token,"seq":True,"forward":0,"turn":0},
            {"token":token,"seq":1,"forward":True,"turn":0},
            {"token":token,"seq":1,"forward":"1","turn":0},
            {"token":"fake","seq":1,"forward":0,"turn":0},
            {"token":token,"seq":1,"forward":0,"turn":0,"topic":"/cmd_vel"},
            {}):
            with self.subTest(input=bad):
                with self.assertRaises(ValueError):
                    validate_command(bad)

    def test_server_wires_only_p6_and_no_generic_ros_topic(self):
        src = (Path(__file__).resolve().parents[1] / "sim_server.py").read_text()
        self.assertIn("from p6_autonomous_mission.cave_simulator import CaveSimulator", src)
        self.assertIn('f"command_topic:={COMMAND}"', src)
        self.assertIn('simulator.command_topic, simulator.odom_topic, simulator.map_topic', src)
        self.assertNotIn("subprocess", src)
        self.assertNotIn("os.system", src)
        self.assertNotIn("create_client(", src)
        self.assertNotIn("'/cmd_vel'", src)
        self.assertNotIn('"/cmd_vel"', src)


if __name__ == "__main__":
    unittest.main()
