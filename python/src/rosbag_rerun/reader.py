"""Stream ROS 1 and ROS 2 bag messages without installing ROS."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator, Sequence

from rosbags.highlevel import AnyReader
from rosbags.typesys.store import Typestore


@dataclass(frozen=True, slots=True)
class BagMessage:
    topic: str
    msg_type: str
    bag_timestamp_ns: int
    message: Any
    decoded: bool


class ROSBagReader:
    """Context-managed, lazy reader for a ROS 1 bag or ROS 2 bag directory.

    The iterator preserves bag order. Message decoding is optional so callers
    can inspect unknown message types or build their own decoders.
    """

    def __init__(self, path: str | Path, *, typestore: Typestore | None = None) -> None:
        self.path = Path(path).expanduser()
        if not self.path.exists():
            raise FileNotFoundError(self.path)
        self._reader = AnyReader([self.path], default_typestore=typestore)
        self._is_open = False

    def __enter__(self) -> ROSBagReader:
        self.open()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def open(self) -> None:
        if not self._is_open:
            self._reader.open()
            self._is_open = True

    def close(self) -> None:
        if self._is_open:
            self._reader.close()
            self._is_open = False

    def _require_open(self) -> None:
        if not self._is_open:
            raise RuntimeError("Open the reader first, preferably with 'with ROSBagReader(...)'.")

    @property
    def topics_and_types(self) -> dict[str, str]:
        self._require_open()
        return {conn.topic: conn.msgtype for conn in self._reader.connections}

    def __iter__(self) -> Iterator[BagMessage]:
        return self.iter_messages()

    def iter_synchronized(
        self,
        groups: Sequence[SyncGroup],
        *,
        start_ns: int | None = None,
        stop_ns: int | None = None,
        deserialize: bool = True,
    ) -> Iterator[SyncedMessage]:
        """Read the union of group topics once and yield synchronized sets."""
        from .sync import iter_synchronized

        topics = tuple(dict.fromkeys(topic for group in groups for topic in group.topics))
        return iter_synchronized(
            self.iter_messages(
                topics=topics,
                start_ns=start_ns,
                stop_ns=stop_ns,
                deserialize=deserialize,
            ),
            groups,
        )

    def iter_messages(
        self,
        *,
        topics: Sequence[str] | None = None,
        start_ns: int | None = None,
        stop_ns: int | None = None,
        deserialize: bool = True,
    ) -> Iterator[BagMessage]:
        """Yield messages in bag-time order; bounds are [start_ns, stop_ns)."""
        self._require_open()
        if start_ns is not None and stop_ns is not None and start_ns >= stop_ns:
            raise ValueError("start_ns must be less than stop_ns")

        selected = None
        if topics is not None:
            topic_set = set(topics)
            selected = [c for c in self._reader.connections if c.topic in topic_set]
            if not selected:
                return

        for connection, timestamp_ns, raw in self._reader.messages(
            connections=selected or (), start=start_ns, stop=stop_ns
        ):
            try:
                message = self._reader.deserialize(raw, connection.msgtype) if deserialize else raw
            except Exception as exc:
                raise ValueError(
                    f"Cannot deserialize {connection.msgtype} on {connection.topic} "
                    f"at {timestamp_ns} ns"
                ) from exc
            yield BagMessage(
                topic=connection.topic,
                msg_type=connection.msgtype,
                bag_timestamp_ns=int(timestamp_ns),
                message=message,
                decoded=deserialize,
            )
