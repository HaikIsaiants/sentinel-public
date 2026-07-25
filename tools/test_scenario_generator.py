import unittest

from sentinel.v1 import sentinel_pb2

import generate_scenarios


class ScenarioGeneratorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.records = generate_scenarios.load_seed_records("development")[:10]
        cls.cbba = {
            record["stratum"]: generate_scenarios.generate_scenario(record, "sentinel_cbba")
            for record in cls.records
        }
        cls.nearest = {
            record["stratum"]: generate_scenarios.generate_scenario(record, "nearest_capable")
            for record in cls.records
        }

    def test_all_strata_are_deterministic_and_paired(self):
        expected = [item["id"] for item in generate_scenarios.STRATA_CONFIG["strata"]]
        self.assertEqual([record["stratum"] for record in self.records], expected)
        for record in self.records:
            cbba = self.cbba[record["stratum"]]
            nearest = self.nearest[record["stratum"]]
            self.assertEqual(generate_scenarios.paired_digest(cbba), generate_scenarios.paired_digest(nearest))
        record = self.records[0]
        repeated = generate_scenarios.generate_scenario(record, "sentinel_cbba")
        self.assertEqual(
            self.cbba[record["stratum"]].SerializeToString(deterministic=True),
            repeated.SerializeToString(deterministic=True),
        )

    def test_stratum_events_are_present(self):
        scenarios = self.cbba
        self.assertTrue(any(event.kind == sentinel_pb2.TAPE_EVENT_KIND_SET_LINK_BLOCKED for event in scenarios["temporary_partition"].events))
        self.assertTrue(any(event.kind == sentinel_pb2.TAPE_EVENT_KIND_SET_REGION_CLOSED for event in scenarios["blocked_route"].events))
        self.assertTrue(any(event.kind == sentinel_pb2.TAPE_EVENT_KIND_RELEASE_TASK for event in scenarios["dynamic_task_arrival"].events))
        self.assertTrue(any(event.kind in {
            sentinel_pb2.TAPE_EVENT_KIND_DISABLE_VEHICLE,
            sentinel_pb2.TAPE_EVENT_KIND_SET_SPEED_PERMILLE,
            sentinel_pb2.TAPE_EVENT_KIND_SET_ENDURANCE_PERMILLE,
        } for event in scenarios["agent_loss_degradation"].events))
        compound = scenarios["compound_disruption"]
        self.assertTrue(any(event.kind == sentinel_pb2.TAPE_EVENT_KIND_SET_LINK_BLOCKED for event in compound.events))
        self.assertTrue(any(event.kind == sentinel_pb2.TAPE_EVENT_KIND_SET_REGION_CLOSED for event in compound.events))
        self.assertTrue(any(event.kind == sentinel_pb2.TAPE_EVENT_KIND_RELEASE_TASK for event in compound.events))
        self.assertTrue(any(event.kind == sentinel_pb2.TAPE_EVENT_KIND_DISABLE_VEHICLE for event in compound.events))

    def test_removal_has_recoverable_orphan(self):
        record = next(record for record in self.records if record["stratum"] == "compound_disruption")
        scenario = self.cbba[record["stratum"]]
        removal = next(event for event in scenario.events if event.kind == sentinel_pb2.TAPE_EVENT_KIND_DISABLE_VEHICLE)
        orphan = next(task for task in scenario.tasks if task.assigned_agent_id == removal.target_id)
        recovery = next(vehicle for vehicle in scenario.vehicles if vehicle.id == "agent-01")
        self.assertTrue(orphan.released)
        self.assertIn(orphan.required_capability, recovery.capabilities)
        self.assertGreaterEqual(recovery.payload_grams, orphan.payload_required_grams)


if __name__ == "__main__":
    unittest.main()
