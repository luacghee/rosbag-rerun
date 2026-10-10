"""Focused checks for ROS image conversion in the viewer."""

import importlib.util
import unittest
from types import SimpleNamespace
from unittest.mock import patch

if importlib.util.find_spec("rerun") and importlib.util.find_spec("numpy"):
    import numpy as np

    from rosbag_rerun import BagMessage
    from rosbag_rerun.viewer import ROS_MESSAGE_TYPES, RerunBagViewer
else:
    np = None
    RerunBagViewer = None


@unittest.skipIf(RerunBagViewer is None, "install the package to test image conversion")
class ViewerTest(unittest.TestCase):
    def test_pose_types_extract_the_pose(self) -> None:
        pose = SimpleNamespace(position=object(), orientation=object())
        messages = {
            ROS_MESSAGE_TYPES["odometry"]: SimpleNamespace(pose=SimpleNamespace(pose=pose)),
            ROS_MESSAGE_TYPES["pose_stamped"]: SimpleNamespace(pose=pose),
            ROS_MESSAGE_TYPES["pose_with_covariance_stamped"]: SimpleNamespace(
                pose=SimpleNamespace(pose=pose)
            ),
        }
        viewer = object.__new__(RerunBagViewer)
        for msg_type, message in messages.items():
            with self.subTest(msg_type=msg_type):
                record = BagMessage("/robot/pose", msg_type, 123, message, True)
                with patch("rosbag_rerun.viewer.rr.set_time"), patch.object(
                    viewer, "_log_pose"
                ) as log_pose:
                    self.assertTrue(viewer.log(record))
                    log_pose.assert_called_once_with("world/robot/pose", pose)

    def test_raw_bgr_image_with_row_padding(self) -> None:
        message = SimpleNamespace(
            encoding="bgr8",
            height=2,
            width=1,
            step=4,
            data=bytes([10, 20, 30, 0, 40, 50, 60, 0]),
        )
        image = RerunBagViewer._raw_image(message)
        np.testing.assert_array_equal(image, [[[30, 20, 10]], [[60, 50, 40]]])

    def test_rejects_incomplete_image_buffer(self) -> None:
        message = SimpleNamespace(
            encoding="rgb8", height=2, width=1, step=3, data=bytes([1, 2, 3])
        )
        self.assertIsNone(RerunBagViewer._raw_image(message))
