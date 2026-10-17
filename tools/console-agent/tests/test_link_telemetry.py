from __future__ import annotations

import unittest
from datetime import datetime, timezone
from types import SimpleNamespace

from console_agent.link_telemetry import LinkTelemetryObserver, TELEMETRY_STREAMS


BASE_TIME = 1_700_000_000
BASE_TEXT = "2023-11-14T22:13:20.000Z"


class FakeMessage:
    def __init__(self, *, seconds: int = BASE_TIME, frame_id: str = "sensor_frame") -> None:
        self.header = SimpleNamespace(
            stamp=SimpleNamespace(sec=seconds, nanosec=0),
            frame_id=frame_id,
        )
        self.width = 100
        self.height = 20

    @property
    def data(self) -> bytes:
        raise AssertionError("telemetry must not read or copy the point payload")


class FakeLivoxCustomMessage:
    def __init__(self) -> None:
        self.header = SimpleNamespace(stamp=SimpleNamespace(sec=BASE_TIME, nanosec=0), frame_id="livox_frame")
        self.point_num = 48_000

    @property
    def points(self) -> list[object]:
        raise AssertionError("telemetry must not traverse Livox point records")


class FakeOccupancyGrid:
    def __init__(self) -> None:
        self.header = SimpleNamespace(stamp=SimpleNamespace(sec=BASE_TIME, nanosec=0), frame_id="map")
        self.info = SimpleNamespace(width=500, height=400)

    @property
    def data(self) -> list[int]:
        raise AssertionError("telemetry must not inspect occupancy values")


class LinkTelemetryObserverTests(unittest.TestCase):
    def setUp(self) -> None:
        self.observer = LinkTelemetryObserver(
            monotonic=lambda: 10.0,
            utc_now=lambda: BASE_TEXT,
        )
        self.observer.set_available_message_types({
            "sensor_msgs/msg/Imu",
            "sensor_msgs/msg/LaserScan",
            "sensor_msgs/msg/PointCloud2",
            "nav_msgs/msg/OccupancyGrid",
            "nav_msgs/msg/Odometry",
            "livox_ros_driver2/msg/CustomMsg",
        })
        self.observer.graph_available(
            domain_id=187,
            observed_at=BASE_TEXT,
            expected_domain_id=None,
            expected_domain_error=None,
        )

    def configure_stream(self, stream_id: str, message_type: str, endpoints: list[dict[str, str]] | None = None) -> int:
        self.observer.update_publishers(stream_id, endpoints or [{
            "name": "synthetic_publisher",
            "namespace": "/synthetic",
            "topic_type": message_type,
        }])
        generation = self.observer.begin_subscription(stream_id, message_type, started_mono=10.0, started_at=BASE_TEXT)
        assert generation is not None
        return generation

    def test_normal_pointcloud_is_measured_without_reading_payload(self) -> None:
        message_type = "sensor_msgs/msg/PointCloud2"
        self.configure_stream("livox_lidar", message_type)
        self.observer.receive("livox_lidar", FakeMessage(), message_type, received_mono=10.0, received_at=BASE_TEXT)
        self.observer.receive("livox_lidar", FakeMessage(seconds=BASE_TIME + 1), message_type,
                              received_mono=10.1, received_at="2023-11-14T22:13:21.000Z")

        reading = self.observer.snapshot(now_mono=10.2, collected_at="2023-11-14T22:13:21.200Z")
        stream = next(item for item in reading["streams"] if item["id"] == "livox_lidar")
        self.assertEqual(reading["domain_id"], 187)
        self.assertEqual(reading["domain_matches_expectation"], None)
        self.assertEqual(stream["status"], "streaming")
        self.assertEqual(stream["topic_type"], message_type)
        self.assertEqual(stream["sample_count"], 2_000)
        self.assertEqual(stream["sample_unit"], "points")
        self.assertAlmostEqual(stream["receive_rate_hz"], 10.0)
        self.assertEqual(stream["age_ms"], 100)
        self.assertEqual(stream["frame_id"], "sensor_frame")
        self.assertEqual(stream["workspace_attribution"], "unknown")
        self.assertEqual(stream["header_to_agent_ms"], 0.0)

    def test_selected_mid360_launch_custom_message_type_is_supported(self) -> None:
        message_type = "livox_ros_driver2/msg/CustomMsg"
        self.configure_stream("livox_lidar", message_type)
        custom = FakeLivoxCustomMessage()
        self.observer.receive("livox_lidar", custom, message_type, received_mono=10.0, received_at=BASE_TEXT)

        stream = next(item for item in self.observer.snapshot(now_mono=10.1)["streams"] if item["id"] == "livox_lidar")
        self.assertEqual(stream["status"], "streaming")
        self.assertEqual(stream["topic_type"], message_type)
        self.assertEqual(stream["sample_count"], 48_000)
        self.assertEqual(stream["frame_id"], "livox_frame")

    def test_stale_stream_is_a_data_observation_not_a_hardware_fault(self) -> None:
        message_type = "sensor_msgs/msg/Imu"
        self.configure_stream("livox_imu", message_type)
        self.observer.receive("livox_imu", FakeMessage(), message_type, received_mono=1.0, received_at=BASE_TEXT)

        stream = next(item for item in self.observer.snapshot(now_mono=2.6)["streams"] if item["id"] == "livox_imu")
        self.assertEqual(stream["status"], "stale")
        self.assertEqual(stream["age_ms"], 1_600)
        self.assertIn("1500 ms", stream["reason"])
        self.assertEqual(stream["workspace_attribution"], "unknown")
        self.assertNotIn("health_state", stream)

    def test_recovery_records_gap_and_returns_to_streaming(self) -> None:
        message_type = "sensor_msgs/msg/LaserScan"
        self.configure_stream("scan", message_type)
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=BASE_TIME, nanosec=0), frame_id="base_frame"),
            ranges=[1.0, 2.0, 3.0],
        )
        self.observer.receive("scan", message, message_type, received_mono=1.0, received_at=BASE_TEXT)
        self.assertEqual(next(item for item in self.observer.snapshot(now_mono=4.0)["streams"] if item["id"] == "scan")["status"], "stale")

        self.observer.receive("scan", message, message_type, received_mono=4.2, received_at=BASE_TEXT)
        stream = next(item for item in self.observer.snapshot(now_mono=4.3)["streams"] if item["id"] == "scan")
        self.assertEqual(stream["status"], "streaming")
        self.assertEqual(stream["gap_events"], 1)
        self.assertEqual(stream["last_gap_ms"], 3_200)
        self.assertEqual(stream["sample_count"], 3)

    def test_absent_expected_stream_and_unconfigured_expectation_remain_non_faulting(self) -> None:
        self.observer.update_publishers("map", [])
        reading = self.observer.snapshot()
        stream = next(item for item in reading["streams"] if item["id"] == "map")
        self.assertEqual(stream["status"], "unavailable")
        self.assertIn("not a hardware-health conclusion", stream["reason"])
        self.assertEqual(reading["expected_domain_id"], None)
        self.assertEqual(reading["domain_matches_expectation"], None)

    def test_domain_mismatch_is_explicit_and_duplicate_workspace_endpoints_are_unattributed(self) -> None:
        message_type = "sensor_msgs/msg/PointCloud2"
        self.observer.graph_available(
            domain_id=7,
            observed_at=BASE_TEXT,
            expected_domain_id=42,
            expected_domain_error=None,
        )
        self.configure_stream("cloud_registered", message_type, endpoints=[
            {"name": "point_lio", "namespace": "/old", "topic_type": message_type},
            {"name": "point_lio", "namespace": "/new", "topic_type": message_type},
        ])
        self.observer.receive("cloud_registered", FakeMessage(), message_type, received_mono=10.0, received_at=BASE_TEXT)

        reading = self.observer.snapshot(now_mono=10.1)
        stream = next(item for item in reading["streams"] if item["id"] == "cloud_registered")
        self.assertEqual(reading["domain_matches_expectation"], False)
        self.assertEqual(reading["domain_id"], 7)
        self.assertEqual(stream["status"], "streaming", "The Agent reports data in its own Domain; the Console gates target-workspace claims.")
        self.assertEqual(stream["publisher_count"], 2)
        self.assertEqual(len(stream["publisher_endpoints"]), 2)
        self.assertEqual(stream["workspace_attribution"], "unknown")

    def test_graph_endpoint_read_error_is_unknown_not_no_publisher(self) -> None:
        self.observer.update_publishers("map", None)
        stream = next(item for item in self.observer.snapshot()["streams"] if item["id"] == "map")
        self.assertEqual(stream["status"], "unknown")
        self.assertIn("endpoint evidence is unavailable", stream["reason"])
        self.assertIsNone(stream["publisher_count"])

    def test_unsupported_or_ambiguous_ros_type_does_not_create_a_stream_claim(self) -> None:
        self.observer.update_publishers("livox_lidar", [
            {"name": "old_driver", "namespace": "/", "topic_type": "sensor_msgs/msg/PointCloud2"},
            {"name": "custom_driver", "namespace": "/", "topic_type": "livox_ros_driver2/msg/CustomMsg"},
        ])
        stream = next(item for item in self.observer.snapshot()["streams"] if item["id"] == "livox_lidar")
        self.assertEqual(stream["status"], "unknown")
        self.assertIsNone(stream["topic_type"])
        self.assertEqual(stream["topic_types"], ["livox_ros_driver2/msg/CustomMsg", "sensor_msgs/msg/PointCloud2"])

    def test_missing_livox_message_type_support_stays_explicitly_unsupported(self) -> None:
        self.observer.set_available_message_types({"sensor_msgs/msg/Imu", "sensor_msgs/msg/PointCloud2"})
        self.observer.update_publishers("livox_lidar", [
            {"name": "livox_lidar_publisher", "namespace": "/", "topic_type": "livox_ros_driver2/msg/CustomMsg"},
        ])
        stream = next(item for item in self.observer.snapshot()["streams"] if item["id"] == "livox_lidar")
        self.assertEqual(stream["status"], "unsupported")
        self.assertIn("type support is not loaded", stream["reason"])
        self.assertEqual(stream["publisher_count"], 1)
        self.assertIsNone(stream["receive_rate_hz"])

    def test_map_sample_size_is_metadata_only(self) -> None:
        message_type = "nav_msgs/msg/OccupancyGrid"
        self.configure_stream("map", message_type)
        message = FakeOccupancyGrid()
        self.observer.receive("map", message, message_type, received_mono=10.0, received_at=BASE_TEXT)
        stream = next(item for item in self.observer.snapshot(now_mono=10.1)["streams"] if item["id"] == "map")
        self.assertEqual(stream["sample_count"], 200_000)
        self.assertEqual(stream["sample_unit"], "cells")

    def test_new_subscription_epoch_does_not_reuse_previous_receive_time(self) -> None:
        message_type = "sensor_msgs/msg/PointCloud2"
        first_generation = self.configure_stream("livox_lidar", message_type)
        self.observer.receive(
            "livox_lidar", FakeMessage(), message_type,
            received_mono=10.0, received_at=BASE_TEXT,
            subscription_generation=first_generation,
        )
        self.observer.receive(
            "livox_lidar", FakeMessage(), message_type,
            received_mono=10.1, received_at="2023-11-14T22:13:20.100Z",
            subscription_generation=first_generation,
        )

        second_generation = self.observer.begin_subscription(
            "livox_lidar", message_type,
            started_mono=11.0,
            started_at="2023-11-14T22:13:21.000Z",
        )
        self.assertEqual(second_generation, first_generation + 1)
        waiting = next(item for item in self.observer.snapshot(now_mono=11.1)["streams"] if item["id"] == "livox_lidar")
        self.assertEqual(waiting["status"], "waiting")
        self.assertIsNone(waiting["age_ms"])
        self.assertIsNone(waiting["receive_rate_hz"])
        self.assertEqual(waiting["receive_count"], 0)
        self.assertEqual(waiting["gap_events"], 0)
        self.assertIsNone(waiting["last_gap_ms"])
        self.assertIsNone(waiting["received_at"])
        self.assertEqual(waiting["previous_received_at"], "2023-11-14T22:13:20.100Z")
        self.assertEqual(waiting["previous_subscription_generation"], first_generation)
        self.assertEqual(waiting["subscription_started_at"], "2023-11-14T22:13:21.000Z")

        # A delayed callback queued by the destroyed subscription cannot contaminate the new epoch.
        self.observer.receive(
            "livox_lidar", FakeMessage(), message_type,
            received_mono=11.2, received_at="2023-11-14T22:13:21.200Z",
            subscription_generation=first_generation,
        )
        still_waiting = next(item for item in self.observer.snapshot(now_mono=11.3)["streams"] if item["id"] == "livox_lidar")
        self.assertEqual(still_waiting["status"], "waiting")
        self.assertEqual(still_waiting["receive_count"], 0)
        self.assertIsNone(still_waiting["received_at"])

        self.observer.receive(
            "livox_lidar", FakeMessage(), message_type,
            received_mono=11.4, received_at="2023-11-14T22:13:21.400Z",
            subscription_generation=second_generation,
        )
        recovered = next(item for item in self.observer.snapshot(now_mono=11.5)["streams"] if item["id"] == "livox_lidar")
        self.assertEqual(recovered["status"], "streaming")
        self.assertEqual(recovered["received_at"], "2023-11-14T22:13:21.400Z")
        self.assertEqual(recovered["previous_received_at"], "2023-11-14T22:13:20.100Z")
        self.assertEqual(recovered["receive_count"], 1)

    def test_cancelled_subscription_keeps_only_explicit_history(self) -> None:
        message_type = "sensor_msgs/msg/Imu"
        generation = self.configure_stream("livox_imu", message_type)
        self.observer.receive(
            "livox_imu", FakeMessage(), message_type,
            received_mono=10.0, received_at=BASE_TEXT,
            subscription_generation=generation,
        )
        self.observer.update_publishers("livox_imu", [])
        self.observer.subscription_unavailable("livox_imu", None, "no publisher is currently visible")

        stream = next(item for item in self.observer.snapshot(now_mono=20.0)["streams"] if item["id"] == "livox_imu")
        self.assertEqual(stream["status"], "unavailable")
        self.assertIsNone(stream["received_at"])
        self.assertIsNone(stream["age_ms"])
        self.assertIsNone(stream["receive_rate_hz"])
        self.assertIsNone(stream["subscription_started_at"])
        self.assertEqual(stream["previous_received_at"], BASE_TEXT)
        self.assertEqual(stream["receive_count"], 0)


if __name__ == "__main__":
    unittest.main()
