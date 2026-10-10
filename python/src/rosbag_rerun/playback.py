"""Optional wall-clock pacing for a stream of bag messages."""

from __future__ import annotations

import math
import time
from collections.abc import Iterable, Iterator
from typing import Callable

from .reader import BagMessage
from .sync import SyncedMessage


def iter_playback(
    messages: Iterable[BagMessage | SyncedMessage],
    *,
    rate: float = 1.0,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> Iterator[BagMessage | SyncedMessage]:
    """Yield messages at recorded intervals; rate=0 disables waiting.

    Pacing uses the first message as its origin and a monotonic clock to avoid
    accumulating processing delays. Messages remain in input order.
    """
    if not math.isfinite(rate) or rate < 0:
        raise ValueError("rate must be a finite, nonnegative number")

    first_timestamp_ns: int | None = None
    start_time: float | None = None
    for message in messages:
        if first_timestamp_ns is None:
            first_timestamp_ns = message.bag_timestamp_ns
            start_time = clock()
        elif rate > 0:
            assert start_time is not None
            offset_s = max(0, message.bag_timestamp_ns - first_timestamp_ns) / 1e9 / rate
            remaining_s = start_time + offset_s - clock()
            if remaining_s > 0:
                sleep(remaining_s)
        yield message
