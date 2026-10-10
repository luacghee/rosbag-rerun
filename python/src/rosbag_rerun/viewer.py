"""Log common decoded ROS messages to Rerun without ROS Python packages."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rerun as rr

from .reader import BagMessage


# ROS message types supported by this viewer.
ROS_MESSAGE_TYPES = {
    "odometry": "nav_msgs/msg/Odometry",
    "pose_stamped": "geometry_msgs/msg/PoseStamped",
    "pose_with_covariance_stamped": "geometry_msgs/msg/PoseWithCovarianceStamped",
    "image": "sensor_msgs/msg/Image",
    "compressed_image": "sensor_msgs/msg/CompressedImage",
}

_POSE_FIELDS = {
    ROS_MESSAGE_TYPES["odometry"]: ("pose", "pose"),
    ROS_MESSAGE_TYPES["pose_stamped"]: ("pose",),
    ROS_MESSAGE_TYPES["pose_with_covariance_stamped"]: ("pose", "pose"),
}
SUPPORTED_MESSAGE_TYPES = frozenset(ROS_MESSAGE_TYPES.values())


class RerunBagViewer:
    """Send pose and image messages to a Rerun viewer or recording file.

    Call :meth:`log` with records from ``ROSBagReader.iter_messages()``. The
    bag timestamp is used for Rerun's ``bag_time`` timeline.
    """

    def __init__(
        self,
        application_id: str = "rosbag_rerun",
        *,
        save_path: str | Path | None = None,
        blueprint_path: str | Path | None = None,
        coordinates: str = "z-up",
    ) -> None:
        if coordinates not in {"z-up", "y-down"}:
            raise ValueError("coordinates must be 'z-up' or 'y-down'")
        if blueprint_path is not None:
            blueprint_path = Path(blueprint_path).expanduser()
            if not blueprint_path.is_file():
                raise FileNotFoundError(blueprint_path)
        rr.init(application_id, spawn=save_path is None)
        if save_path is not None:
            rr.save(str(save_path))
        if blueprint_path is not None:
            rr.log_file_from_path(blueprint_path)
        orientation = (
            rr.ViewCoordinates.RIGHT_HAND_Z_UP
            if coordinates == "z-up"
            else rr.ViewCoordinates.RIGHT_HAND_Y_DOWN
        )
        rr.log("world", orientation, static=True)
        self._pose_entities: set[str] = set()

    def log(self, record: BagMessage) -> bool:
        """Log a supported decoded message; return False for unsupported data."""
        if not record.decoded:
            raise ValueError("RerunBagViewer requires deserialized messages")
        if record.msg_type not in SUPPORTED_MESSAGE_TYPES:
            return False

        rr.set_time("bag_time", timestamp=np.datetime64(record.bag_timestamp_ns, "ns"))
        topic_path = record.topic.strip("/") or "root"
        pose_fields = _POSE_FIELDS.get(record.msg_type)
        if pose_fields is not None:
            pose = record.message
            for field in pose_fields:
                pose = getattr(pose, field)
            self._log_pose(f"world/{topic_path}", pose)
        elif record.msg_type == ROS_MESSAGE_TYPES["image"]:
            image = self._raw_image(record.message)
            if image is None:
                return False
            rr.log(f"images/{topic_path}", rr.Image(image))
        elif record.msg_type == ROS_MESSAGE_TYPES["compressed_image"]:
            image = self._compressed_image(record.message)
            if image is None:
                return False
            rr.log(f"images/{topic_path}", image)
        else:
            return False
        return True

    def _log_pose(self, entity: str, pose: object) -> None:
        if entity not in self._pose_entities:
            rr.log(
                entity,
                rr.Arrows3D(
                    vectors=[[0.2, 0, 0], [0, 0.2, 0], [0, 0, 0.2]],
                    colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                ),
                static=True,
            )
            self._pose_entities.add(entity)
        position = pose.position
        rotation = pose.orientation
        rr.log(
            entity,
            rr.Transform3D(
                translation=[position.x, position.y, position.z],
                quaternion=rr.Quaternion(xyzw=[rotation.x, rotation.y, rotation.z, rotation.w]),
            ),
        )

    @staticmethod
    def _raw_image(message: object) -> np.ndarray | None:
        channels = {"mono8": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
        encoding = message.encoding.lower()
        if encoding not in channels:
            return None
        height, width, step = int(message.height), int(message.width), int(message.step)
        count = channels[encoding]
        if height <= 0 or width <= 0 or step < width * count:
            return None
        data = np.frombuffer(message.data, dtype=np.uint8)
        if data.size < height * step:
            return None
        image = data[: height * step].reshape(height, step)[:, : width * count]
        image = image.reshape(height, width, count)
        if encoding.startswith("bgr"):
            image = image[..., [2, 1, 0] + ([3] if count == 4 else [])]
        if count == 1:
            image = image[:, :, 0]
        return np.ascontiguousarray(image)

    @staticmethod
    def _compressed_image(message: object) -> rr.EncodedImage | None:
        format_name = message.format.lower()
        if "jpeg" in format_name or "jpg" in format_name:
            media_type = "image/jpeg"
        elif "png" in format_name and "compresseddepth" not in format_name:
            media_type = "image/png"
        else:
            return None
        return rr.EncodedImage(contents=bytes(message.data), media_type=media_type)
