import unittest

from rosbag_rerun import BagMessage, iter_playback


class PlaybackTest(unittest.TestCase):
    def test_rate_scales_recorded_intervals(self) -> None:
        now = [0.0]
        sleeps = []

        def sleep(seconds: float) -> None:
            sleeps.append(seconds)
            now[0] += seconds

        messages = [
            BagMessage("/a", "T", timestamp, object(), True)
            for timestamp in (100, 1_000_000_100, 2_000_000_100)
        ]
        self.assertEqual(
            list(iter_playback(messages, rate=2, clock=lambda: now[0], sleep=sleep)),
            messages,
        )
        self.assertEqual(sleeps, [0.5, 0.5])

    def test_rate_zero_has_no_wait(self) -> None:
        message = BagMessage("/a", "T", 0, object(), True)
        self.assertEqual(list(iter_playback([message], rate=0, sleep=lambda _: self.fail())), [message])
        with self.assertRaises(ValueError):
            list(iter_playback([message], rate=-1))
