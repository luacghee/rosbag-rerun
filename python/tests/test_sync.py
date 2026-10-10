import unittest

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
