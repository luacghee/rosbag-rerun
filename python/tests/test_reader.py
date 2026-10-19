from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

from rosbag_rerun import ROSBagReader, SyncGroup


class ROSBagReaderTest(unittest.TestCase):
    def test_streams_decoded_and_raw_messages_with_filters(self) -> None:
        store = get_typestore(Stores.ROS1_NOETIC)
        string_type = store.types["std_msgs/msg/String"]
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "small.bag"
            with Writer(path) as writer:
                first = writer.add_connection("/first", string_type.__msgtype__, typestore=store)
                second = writer.add_connection("/second", string_type.__msgtype__, typestore=store)
                for connection, timestamp, value in (
                    (first, 1_000_000_000, "one"),
                    (second, 2_000_000_000, "two"),
                    (first, 3_000_000_000, "three"),
                ):
                    writer.write(
                        connection,
                        timestamp,
                        store.serialize_ros1(string_type(value), string_type.__msgtype__),
                    )

            with ROSBagReader(path) as reader:
                self.assertEqual(set(reader.topics_and_types), {"/first", "/second"})
                self.assertEqual([msg.message.data for msg in reader], ["one", "two", "three"])
                filtered = list(reader.iter_messages(topics=["/first"], start_ns=2_000_000_000))
                self.assertEqual([msg.message.data for msg in filtered], ["three"])
                raw = list(reader.iter_messages(topics=["/second"], deserialize=False))
                self.assertEqual(len(raw), 1)
                self.assertFalse(raw[0].decoded)
                self.assertIsInstance(raw[0].message, bytes)
                self.assertEqual(list(reader.iter_messages(topics=["/missing"])), [])
                synced = list(
                    reader.iter_synchronized(
                        [
                            SyncGroup(
                                name="pair",
                                topics=("/first", "/second"),
                                reference_topic="/first",
                                tolerance_ns=1_000_000_000,
                                arrival_wait_ns=1_000_000_000,
                            )
                        ]
                    )
                )
                self.assertEqual(len(synced), 1)
                self.assertEqual(synced[0].timestamp_ns, 1_000_000_000)
                self.assertEqual(synced[0].messages["/second"].message.data, "two")

            with self.assertRaises(RuntimeError):
                list(reader.iter_messages())
