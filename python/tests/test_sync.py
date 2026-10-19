import unittest
from types import SimpleNamespace

from rosbag_rerun import BagMessage, MessageSynchronizer, SyncGroup, iter_synchronized


def msg(topic: str, timestamp_ns: int, payload: object | None = None) -> BagMessage:
    return BagMessage(topic, "test/Type", timestamp_ns, payload, True)


class SynchronizerTest(unittest.TestCase):
    def test_topic_can_join_two_independent_groups(self) -> None:
        groups = [
            SyncGroup(
                name="camera_imu",
                topics=("/camera", "/imu"),
                reference_topic="/camera",
                tolerance_ns=10,
            ),
            SyncGroup(
                name="camera_pose",
                topics=("/camera", "/pose"),
                reference_topic="/camera",
                tolerance_ns=10,
            ),
        ]
        camera = msg("/camera", 100)
        stream = [msg("/imu", 95), camera, msg("/pose", 104), msg("/other", 120)]
        result = list(iter_synchronized(stream, groups))

        self.assertEqual([bundle.group for bundle in result], ["camera_imu", "camera_pose"])
        self.assertIs(result[0].messages["/camera"], camera)
        self.assertIs(result[1].messages["/camera"], camera)
        self.assertEqual(result[0].messages["/imu"].bag_timestamp_ns, 95)
        self.assertEqual(result[1].messages["/pose"].bag_timestamp_ns, 104)

    def test_waits_for_closer_message_and_consumes_once_per_group(self) -> None:
        group = SyncGroup(
            name="pair",
            topics=("/camera", "/imu"),
            reference_topic="/camera",
            tolerance_ns=10,
        )
        stream = [
            msg("/imu", 98),
            msg("/camera", 100),
            msg("/imu", 101),
            msg("/camera", 102),
            msg("/other", 113),
        ]
        result = list(iter_synchronized(stream, [group]))
        self.assertEqual([bundle.messages["/imu"].bag_timestamp_ns for bundle in result], [101, 98])
        self.assertEqual([bundle.timestamp_ns for bundle in result], [100, 102])

    def test_shared_nonreference_topic_is_independent(self) -> None:
        imu = msg("/imu", 100)
        groups = [
            SyncGroup(
                name="camera_imu",
                topics=("/camera", "/imu"),
                reference_topic="/camera",
                tolerance_ns=5,
            ),
            SyncGroup(
                name="pose_imu",
                topics=("/pose", "/imu"),
                reference_topic="/pose",
                tolerance_ns=5,
            ),
        ]
        result = list(
            iter_synchronized(
                [imu, msg("/camera", 101), msg("/pose", 102)], groups
            )
        )
        self.assertEqual(len(result), 2)
        self.assertIs(result[0].messages["/imu"], imu)
        self.assertIs(result[1].messages["/imu"], imu)

    def test_flushes_at_end_and_drops_incomplete_sets(self) -> None:
        group = SyncGroup(
            name="pair",
            topics=("/camera", "/imu"),
            reference_topic="/camera",
            tolerance_ns=10,
        )
        self.assertEqual(
            len(list(iter_synchronized([msg("/camera", 100), msg("/imu", 105)], [group]))),
            1,
        )
        self.assertEqual(list(iter_synchronized([msg("/camera", 100)], [group])), [])

    def test_rejects_out_of_order_input_and_duplicate_group_names(self) -> None:
        group = SyncGroup(
            name="pair",
            topics=("/camera", "/imu"),
            reference_topic="/camera",
            tolerance_ns=10,
        )
        sync = MessageSynchronizer([group])
        sync.add(msg("/camera", 100))
        with self.assertRaises(ValueError):
            sync.add(msg("/imu", 99))
        with self.assertRaises(ValueError):
            MessageSynchronizer([group, group])
        with self.assertRaises(ValueError):
            SyncGroup(name="empty", topics=(), reference_topic="/camera")


class ArrivalWaitTest(unittest.TestCase):
    @staticmethod
    def record(topic, bag_ns, header_ns):
        sec, nanosec = divmod(header_ns, 1_000_000_000)
        payload = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)))
        return msg(topic, bag_ns, payload)

    def group(self, **options):
        return SyncGroup(name='rgbd', topics=('rgb', 'depth'), reference_topic='rgb',
                         timestamp_source='header', **options)

    def test_delayed_exact_pairs_keep_bag_order_without_widening_tolerance(self):
        records = []
        for i in range(4):
            stamp = 1_000_000_000 + i * 33_000_000
            records.extend([self.record('rgb', stamp, stamp),
                            self.record('depth', stamp + 55_000_000, stamp)])
        records.sort(key=lambda record: record.bag_timestamp_ns)
        result = list(iter_synchronized(records, [self.group()]))
        self.assertEqual([bundle.timestamp_ns for bundle in result],
                         [1_000_000_000 + i * 33_000_000 for i in range(4)])
        self.assertTrue(all(bundle.messages['rgb'].message.header.stamp ==
                            bundle.messages['depth'].message.header.stamp for bundle in result))
        legacy = list(iter_synchronized(records, [self.group(arrival_wait_ns=30_000_000)]))
        self.assertLess(len(legacy), len(result))

    def test_header_tolerance_still_rejects_bad_timestamp_matches(self):
        records = [self.record('rgb', 0, 1_000_000_000),
                   self.record('depth', 50_000_000, 1_031_000_000)]
        self.assertEqual(list(iter_synchronized(records, [self.group()])), [])

    def test_late_partner_is_rejected_even_when_it_advances_the_deadline(self):
        records = [self.record('rgb', 0, 1_000_000_000),
                   self.record('depth', 200_000_001, 1_000_000_000)]
        self.assertEqual(list(iter_synchronized(records, [self.group()])), [])

    def test_partner_at_deadline_is_accepted(self):
        records = [self.record('rgb', 0, 1_000_000_000),
                   self.record('depth', 200_000_000, 1_000_000_000)]
        self.assertEqual(len(list(iter_synchronized(records, [self.group()]))), 1)

    def test_unrelated_topics_advance_timeout_without_blocking_later_frames(self):
        records = [self.record('rgb', 0, 1_000_000_000),
                   msg('imu', 200_000_000),
                   self.record('depth', 210_000_000, 1_000_000_000),
                   self.record('rgb', 220_000_000, 1_220_000_000),
                   self.record('depth', 250_000_000, 1_220_000_000)]
        result = list(iter_synchronized(records, [self.group()]))
        self.assertEqual([bundle.timestamp_ns for bundle in result], [1_220_000_000])

    def test_waits_for_closer_partner_and_consumes_it_once(self):
        records = [self.record('rgb', 0, 1_000_000_000),
                   self.record('depth', 10_000_000, 990_000_000),
                   self.record('depth', 55_000_000, 1_000_000_000)]
        result = list(iter_synchronized(records, [self.group()]))
        self.assertEqual(result[0].messages['depth'].bag_timestamp_ns, 55_000_000)

    def test_explicit_reordering_keeps_its_own_header_watermark(self):
        records = [self.record('rgb', 0, 1_200_000_000),
                   self.record('depth', 10_000_000, 1_200_000_000),
                   self.record('rgb', 20_000_000, 1_000_000_000),
                   self.record('depth', 30_000_000, 1_000_000_000)]
        group = self.group(reorder_by_timestamp=True, max_out_of_order_ns=200_000_000,
                           arrival_wait_ns=1)
        result = list(iter_synchronized(records, [group]))
        self.assertEqual([bundle.timestamp_ns for bundle in result], [1_000_000_000, 1_200_000_000])

    def test_invalid_arrival_wait_is_rejected(self):
        for value in (-1, 1.5, True, float('inf')):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'arrival_wait_ns'):
                self.group(arrival_wait_ns=value)
