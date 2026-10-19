from __future__ import annotations

import unittest

from scripts.benchmark_link_telemetry import (
    MIN_ACCEPTED_RATE_HZ,
    _ReceiveCounter,
    _diagnose_pipeline,
    _event_rate_hz,
    _summarize_deliveries,
)


class PointCloudBenchmarkMetricsTests(unittest.TestCase):
    def test_publisher_probe_and_agent_counts_are_independent(self) -> None:
        result = _summarize_deliveries(
            [0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
            [0.0, 0.1, 0.3, 0.4],
            agent_received_count=3,
            start=0.0,
            end=0.5,
            probe_sequences=[1, 2, 4, 5],
        )
        self.assertEqual(result["publisher_count"], 6)
        self.assertEqual(result["probe_count"], 4)
        self.assertEqual(result["agent_count"], 3)
        self.assertEqual(result["publisher_probe_missing_lower_bound"], 2)
        self.assertEqual(result["publisher_probe_sequence_delivered_count"], 4)
        self.assertEqual(result["publisher_probe_sequence_missing_count"], 2)
        self.assertEqual(result["probe_agent_missing_lower_bound"], 1)

    def test_publisher_timer_rate_uses_observed_publish_timestamps(self) -> None:
        self.assertEqual(_event_rate_hz([0.0, 0.1, 0.2, 0.4], 0.0, 0.4), 7.5)
        self.assertEqual(_event_rate_hz([0.0], 0.0, 1.0), 0.0)

    def test_generation_shortfall_is_identified_before_delivery_stages(self) -> None:
        result = _diagnose_pipeline(4.9, 4.9, 4.9, ci_pressure_observed=False)
        self.assertEqual(result["first_stage_below_threshold"], "message_generation_below_5_hz")
        self.assertFalse(result["publisher_stage_at_least_5_hz"])

    def test_dds_probe_shortfall_is_separated_from_agent_rate(self) -> None:
        result = _diagnose_pipeline(10.0, 4.0, 3.5, ci_pressure_observed=False)
        self.assertEqual(result["first_stage_below_threshold"], "dds_delivery_or_ci_pressure_below_5_hz")

    def test_agent_shortfall_is_identified_when_publisher_and_probe_are_healthy(self) -> None:
        result = _diagnose_pipeline(10.0, 9.8, 4.9, ci_pressure_observed=False)
        self.assertEqual(result["first_stage_below_threshold"], "agent_receive_path_below_5_hz")
        self.assertFalse(result["agent_receive_p05_at_least_5_hz"])

    def test_ci_throttling_is_retained_as_a_confounder(self) -> None:
        result = _diagnose_pipeline(3.0, 3.0, 2.0, ci_pressure_observed=True)
        self.assertTrue(result["ci_resource_pressure_observed"])
        self.assertIn("cannot be attributed to the Agent alone", result["interpretation"])

    def test_five_hz_acceptance_threshold_is_not_lowered(self) -> None:
        self.assertEqual(MIN_ACCEPTED_RATE_HZ, 5.0)
        self.assertFalse(_diagnose_pipeline(5.0, 5.0, 4.99, ci_pressure_observed=False)[
            "agent_receive_p05_at_least_5_hz"
        ])

    def test_agent_callback_count_survives_subscription_generation_changes(self) -> None:
        counter = _ReceiveCounter()
        counter.observe(1, 10)
        counter.observe(1, 13)
        counter.observe(2, 2)
        counter.observe(2, 5)
        self.assertEqual(counter.total, 8)


if __name__ == "__main__":
    unittest.main()
