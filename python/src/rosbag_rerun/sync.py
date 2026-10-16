"""Approximate time synchronization for a stream of bag messages."""

from __future__ import annotations

from bisect import insort
from collections.abc import Iterable, Iterator, Mapping
from dataclasses import dataclass

from .reader import BagMessage


@dataclass(frozen=True, slots=True)
class SyncGroup:
    """A named, independent set of topics matched around a reference topic."""

    name: str
    topics: tuple[str, ...]
    reference_topic: str
    tolerance_ns: int = 30_000_000
    max_queue_size: int = 1000
    timestamp_source: str = "bag"
    max_out_of_order_ns: int = 200_000_000

    def __post_init__(self) -> None:
        object.__setattr__(self, "topics", tuple(self.topics))
        if not self.name:
            raise ValueError("sync group name must not be empty")
        if not self.topics or len(set(self.topics)) != len(self.topics):
            raise ValueError("sync group topics must be nonempty and unique")
        if self.reference_topic not in self.topics:
            raise ValueError("reference_topic must be in topics")
        if self.tolerance_ns < 0:
            raise ValueError("tolerance_ns must be nonnegative")
        if self.max_queue_size < 1:
            raise ValueError("max_queue_size must be positive")
        if self.timestamp_source not in ("bag", "header"):
            raise ValueError("timestamp_source must be 'bag' or 'header'")
        if self.max_out_of_order_ns < 0:
            raise ValueError("max_out_of_order_ns must be nonnegative")


@dataclass(frozen=True, slots=True)
class SyncedMessage:
    group: str
    timestamp_ns: int
    bag_timestamp_ns: int
    messages: Mapping[str, BagMessage]


def _timestamp_ns(message: BagMessage, source: str) -> int:
    if source == "bag":
        return message.bag_timestamp_ns
    if not message.decoded:
        raise ValueError("header synchronization requires deserialized messages")
    try:
        stamp = message.message.header.stamp
        sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError(f"Missing header timestamp on {message.topic}") from exc
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError(f"Invalid header timestamp on {message.topic}")
    return sec * 1_000_000_000 + nanosec


class _GroupState:
    def __init__(self, config: SyncGroup) -> None:
        self.config = config
        # Sorted by the selected event time, even when bag order differs.
        self.queues: dict[str, list[tuple[int, int, BagMessage]]] = {
            topic: [] for topic in config.topics
        }
        self.latest_ns: int | None = None
        self.sequence = 0

    def add(self, message: BagMessage) -> None:
        if message.topic not in self.queues:
            return
        timestamp_ns = _timestamp_ns(message, self.config.timestamp_source)
        if self.latest_ns is not None and timestamp_ns < self.latest_ns - self.config.max_out_of_order_ns:
            if self.config.timestamp_source == "header":
                raise ValueError(
                    f"Header timestamp on {message.topic} arrived more than "
                    f"{self.config.max_out_of_order_ns} ns out of order; increase max_out_of_order_ns"
                )
        self.latest_ns = max(timestamp_ns, self.latest_ns or timestamp_ns)
        insort(self.queues[message.topic], (timestamp_ns, self.sequence, message))
        self.sequence += 1
        if len(self.queues[message.topic]) > self.config.max_queue_size:
            self.queues[message.topic].pop(0)

    def drain(self, finish: bool = False) -> list[SyncedMessage]:
        config = self.config
        if self.latest_ns is None:
            return []
        watermark_ns = (float("inf") if finish else
                        self.latest_ns - config.tolerance_ns -
                        (config.max_out_of_order_ns if config.timestamp_source == "header" else 0))
        reference_queue = self.queues[config.reference_topic]
        ready: list[SyncedMessage] = []
        while reference_queue and reference_queue[0][0] <= watermark_ns:
            timestamp_ns, _, reference = reference_queue.pop(0)
            matched: dict[str, BagMessage] = {config.reference_topic: reference}
            selected_indices: dict[str, int] = {}
            for topic in config.topics:
                if topic == config.reference_topic:
                    continue
                queue = self.queues[topic]
                candidates = (
                    (index, item) for index, item in enumerate(queue)
                    if abs(item[0] - timestamp_ns) <= config.tolerance_ns
                )
                nearest = min(
                    candidates,
                    key=lambda pair: (abs(pair[1][0] - timestamp_ns), pair[1][0]),
                    default=None,
                )
                if nearest is None:
                    break
                selected_indices[topic] = nearest[0]
                matched[topic] = nearest[1][2]
            if len(matched) == len(config.topics):
                for topic, index in selected_indices.items():
                    del self.queues[topic][index]
                ready.append(SyncedMessage(
                    group=config.name,
                    timestamp_ns=timestamp_ns,
                    bag_timestamp_ns=reference.bag_timestamp_ns,
                    messages={topic: matched[topic] for topic in config.topics},
                ))

        if not finish:
            cutoff_ns = watermark_ns - config.tolerance_ns
            for topic, queue in self.queues.items():
                if topic != config.reference_topic:
                    while queue and queue[0][0] < cutoff_ns:
                        queue.pop(0)
        return ready


class MessageSynchronizer:
    """Synchronize one chronological stream into independent named groups.

    Each group owns its own queues. A message can therefore be used once per
    group, including when its topic participates in several groups.
    """

    def __init__(self, groups: Iterable[SyncGroup]) -> None:
        configs = tuple(groups)
        if not configs:
            raise ValueError("at least one sync group is required")
        if len({group.name for group in configs}) != len(configs):
            raise ValueError("sync group names must be unique")
        self._groups = [_GroupState(group) for group in configs]
        self._last_timestamp_ns: int | None = None

    def add(self, message: BagMessage) -> list[SyncedMessage]:
        timestamp_ns = message.bag_timestamp_ns
        if self._last_timestamp_ns is not None and timestamp_ns < self._last_timestamp_ns:
            raise ValueError("messages must arrive in nondecreasing bag timestamp order")
        self._last_timestamp_ns = timestamp_ns
        for group in self._groups:
            group.add(message)
        return self._drain()

    def finish(self) -> list[SyncedMessage]:
        """Emit any complete groups remaining at the end of the input stream."""
        return self._drain(finish=True)

    def _drain(self, finish: bool = False) -> list[SyncedMessage]:
        ready = [item for group in self._groups for item in group.drain(finish)]
        ready.sort(key=lambda item: item.timestamp_ns)
        return ready


def iter_synchronized(
    messages: Iterable[BagMessage],
    groups: Iterable[SyncGroup],
) -> Iterator[SyncedMessage]:
    """Yield complete groups in reference-time order; unmatched data is skipped."""
    synchronizer = MessageSynchronizer(groups)
    for message in messages:
        yield from synchronizer.add(message)
    yield from synchronizer.finish()
