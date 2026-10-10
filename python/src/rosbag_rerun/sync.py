"""Approximate time synchronization for a stream of bag messages."""

from __future__ import annotations

from collections import deque
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


@dataclass(frozen=True, slots=True)
class SyncedMessage:
    group: str
    timestamp_ns: int
    bag_timestamp_ns: int
    messages: Mapping[str, BagMessage]


class _GroupState:
    def __init__(self, config: SyncGroup) -> None:
        self.config = config
        self.queues: dict[str, deque[BagMessage]] = {
            topic: deque(maxlen=config.max_queue_size) for topic in config.topics
        }

    def add(self, message: BagMessage) -> None:
        if message.topic in self.queues:
            self.queues[message.topic].append(message)

    def drain(self, watermark_ns: float) -> list[SyncedMessage]:
        config = self.config
        reference_queue = self.queues[config.reference_topic]
        ready: list[SyncedMessage] = []
        while reference_queue and reference_queue[0].bag_timestamp_ns <= watermark_ns:
            reference = reference_queue.popleft()
            timestamp_ns = reference.bag_timestamp_ns
            matched: dict[str, BagMessage] = {config.reference_topic: reference}
            selected_indices: dict[str, int] = {}
            for topic in config.topics:
                if topic == config.reference_topic:
                    continue
                queue = self.queues[topic]
                candidates = (
                    (index, item) for index, item in enumerate(queue)
                    if abs(item.bag_timestamp_ns - timestamp_ns) <= config.tolerance_ns
                )
                nearest = min(
                    candidates,
                    key=lambda pair: (
                        abs(pair[1].bag_timestamp_ns - timestamp_ns),
                        pair[1].bag_timestamp_ns,
                    ),
                    default=None,
                )
                if nearest is None:
                    break
                selected_indices[topic], matched[topic] = nearest
            if len(matched) == len(config.topics):
                for topic, index in selected_indices.items():
                    del self.queues[topic][index]
                ready.append(
                    SyncedMessage(
                        group=config.name,
                        timestamp_ns=timestamp_ns,
                        bag_timestamp_ns=timestamp_ns,
                        messages={topic: matched[topic] for topic in config.topics},
                    )
                )

        # No future reference can match messages older than this bound.
        if watermark_ns != float("inf"):
            cutoff_ns = watermark_ns - config.tolerance_ns
            for topic, queue in self.queues.items():
                if topic != config.reference_topic:
                    while queue and queue[0].bag_timestamp_ns < cutoff_ns:
                        queue.popleft()
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
        self._max_tolerance_ns = max(group.tolerance_ns for group in configs)
        self._last_timestamp_ns: int | None = None

    def add(self, message: BagMessage) -> list[SyncedMessage]:
        timestamp_ns = message.bag_timestamp_ns
        if self._last_timestamp_ns is not None and timestamp_ns < self._last_timestamp_ns:
            raise ValueError("messages must arrive in nondecreasing bag timestamp order")
        self._last_timestamp_ns = timestamp_ns
        for group in self._groups:
            group.add(message)
        watermark_ns = timestamp_ns - self._max_tolerance_ns
        return self._drain(watermark_ns)

    def finish(self) -> list[SyncedMessage]:
        """Emit any complete groups remaining at the end of the input stream."""
        return self._drain(float("inf"))

    def _drain(self, watermark_ns: float) -> list[SyncedMessage]:
        ready = [item for group in self._groups for item in group.drain(watermark_ns)]
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
