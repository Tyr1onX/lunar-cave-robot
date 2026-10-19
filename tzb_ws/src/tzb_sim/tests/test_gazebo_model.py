import tempfile
import unittest
from pathlib import Path

from tzb_sim.gazebo_model import build_gazebo_urdf


class GazeboModelDescriptionTests(unittest.TestCase):
    def test_gazebo_and_rviz_mesh_uris_are_compatible(self):
        urdf = [
            '<robot name="tzb">',
            '  <link name="base_link"><visual><geometry><mesh filename="package://tzb_description/meshes/base_link.STL"/></geometry></visual></link>',
        ]
        for name in ("J1_joint", "J2_joint", "J3_joint", "J4_joint",
                     "J5_joint", "J6_joint", "Claw_left_joint", "claw_right_joint"):
            lower, upper = ("0", "1") if name != "claw_right_joint" else ("0", "1")
            urdf.append(
                f'  <joint name="{name}" type="revolute"><parent link="base_link"/><child link="{name}_link"/><limit lower="{lower}" upper="{upper}" effort="2" velocity="1"/></joint>')
            urdf.append(f'  <link name="{name}_link"/>')
        urdf.append('</robot>')
        with tempfile.TemporaryDirectory() as directory:
            description = Path(directory) / "tzb_description"
            (description / "urdf").mkdir(parents=True)
            source = description / "urdf" / "tzb.urdf"
            source.write_text("\n".join(urdf), encoding="utf-8")
            gazebo = build_gazebo_urdf(str(source), str(Path(directory) / "controllers.yaml"))
            rviz = build_gazebo_urdf(
                str(source), str(Path(directory) / "controllers.yaml"),
                preserve_package_meshes=True)
        self.assertIn(str(Path(directory) / "tzb_description/meshes/base_link.STL"), gazebo)
        self.assertIn("package://tzb_description/meshes/base_link.STL", rviz)
        import xml.etree.ElementTree as ET
        simulated = ET.fromstring(gazebo)
        control = simulated.find("ros2_control")
        joints = {joint.get("name"): joint for joint in control.findall("joint")}
        j2_initial = joints["J2_joint"].find("state_interface[@name='position']/param[@name='initial_value']")
        j3_initial = joints["J3_joint"].find("state_interface[@name='position']/param[@name='initial_value']")
        self.assertEqual(j2_initial.text, "0.800000")
        self.assertEqual(j3_initial.text, "-2.300000")
        j2_interface = joints["J2_joint"].find("command_interface")
        self.assertEqual(j2_interface.get("name"), "position")
        j2_limits = {param.get("name"): param.text for param in j2_interface.findall("param")}
        self.assertEqual(j2_limits, {"min": "0", "max": "1"})
        gripper_interface = joints["Claw_left_joint"].find("command_interface")
        self.assertEqual(gripper_interface.get("name"), "effort")
        gripper_limits = {param.get("name"): param.text for param in gripper_interface.findall("param")}
        self.assertEqual(gripper_limits, {"min": "-2.0", "max": "2"})
        self.assertEqual(simulated.find("joint[@name='J4_joint']/dynamics").get("damping"), "0.100")
        self.assertEqual(simulated.find("joint[@name='J6_joint']/dynamics").get("damping"), "0.100")

    def test_world_has_a_physical_place_pad_covering_the_release_zone(self):
        import xml.etree.ElementTree as ET
        world_path = Path(__file__).resolve().parents[1] / "config" / "gazebo_world.sdf"
        world = ET.parse(world_path).getroot()
        pad = world.find(".//model[@name='place_pad']")
        self.assertIsNotNone(pad)
        self.assertEqual(pad.findtext("static"), "true")
        pose = [float(value) for value in pad.findtext("pose").split()[:3]]
        size = [float(value) for value in pad.find(".//collision/geometry/box/size").text.split()]
        self.assertAlmostEqual(pose[0], 0.13)
        self.assertAlmostEqual(pose[1], -0.54)
        self.assertGreaterEqual(size[0], 0.50)
        self.assertGreaterEqual(size[1], 0.24)
        # Predicted held-object release point is supported by the pad.
        self.assertLess(abs(-0.00656 - pose[0]) + 0.025, size[0] / 2)
        self.assertLess(abs(-0.51066 - pose[1]) + 0.025, size[1] / 2)
        # The pad is physically clear of the base mesh and wide enough for observed jaw-release drift.
        self.assertLess(pose[1] + size[1] / 2, -0.40087)
        self.assertLess(abs(0.1975 - pose[0]) + 0.025, size[0] / 2)
        self.assertLess(abs(-0.5869 - pose[1]) + 0.025, size[1] / 2)

    def test_rviz_subscribes_to_live_p2_object_and_planner_topics(self):
        import yaml
        config = Path(__file__).resolve().parents[1] / "config" / "physical_grasp.rviz"
        rviz = yaml.safe_load(config.read_text(encoding="utf-8"))
        displays = rviz["Visualization Manager"]["Displays"]
        values = []
        for display in displays:
            for field in ("Marker Topic", "Topic"):
                topic = display.get(field, {})
                if isinstance(topic, dict) and topic.get("Value"):
                    values.append(topic["Value"])
        self.assertIn("/p2_gazebo/physical_object_marker", values)
        self.assertIn("/tzb_catch/reference_frames", values)
        self.assertIn("/lite_motion_planner/collision_markers", values)
        self.assertFalse(any("/p2_offline/" in value for value in values))

    def test_sim_grasp_offset_is_separate_from_perceived_object_pose(self):
        import yaml
        config = Path(__file__).resolve().parents[1] / "config" / "sim_overrides.yaml"
        params = yaml.safe_load(config.read_text(encoding="utf-8"))
        catch = params["tzb_catch"]["ros__parameters"]
        self.assertEqual(catch["tzb.grasp_tcp_offset_xyz"], [0.0, 0.0, 0.02])
        self.assertAlmostEqual(catch["tzb.release_settle_sec"], 1.25)
        self.assertTrue(catch["tzb.force_target_roll"])
        self.assertTrue(catch["tzb.force_target_pitch"])
        self.assertTrue(catch["tzb.force_target_yaw"])
        self.assertFalse(catch["tzb.position_only_target"])


if __name__ == "__main__":
    unittest.main()
