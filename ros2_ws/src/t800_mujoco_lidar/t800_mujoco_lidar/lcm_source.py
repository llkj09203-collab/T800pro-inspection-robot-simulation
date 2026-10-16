"""Validated latest-state receiver for the SDK's existing LCM snapshot channel."""

import json
import threading
import time


class LatestState:
    def __init__(self, timeout):
        self.timeout = timeout
        self._lock = threading.Lock()
        self._state = None
        self.rejected = 0

    def receive(self, _channel, payload):
        try:
            if len(payload) > 128000:
                raise ValueError("state payload is too large")
            state = json.loads(payload)
            age = time.time() - float(state["stamp"])
            if state["version"] != 3:
                raise ValueError("state protocol mismatch")
            scenario = state["scenario"]
            if not isinstance(scenario, str) or not scenario:
                raise ValueError("state has no scenario name")
            if not -0.1 <= age <= self.timeout:
                raise ValueError("state is stale")
            with self._lock:
                if self._state is None or state["stamp"] > self._state["stamp"]:
                    self._state = state
        except (ValueError, TypeError, KeyError, json.JSONDecodeError):
            self.rejected += 1

    def get(self):
        with self._lock:
            return self._state


class LcmStateSource:
    def __init__(self, url, channel, timeout):
        import lcm

        self.receiver = lcm.LCM(url)
        self.publisher = lcm.LCM(url)
        self.latest = LatestState(timeout)
        self.receiver.subscribe(channel, self.latest.receive).set_queue_capacity(1)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            self.receiver.handle_timeout(100)

    def get(self):
        return self.latest.get()

    def publish_debug_rays(self, payload):
        self.publisher.publish("lidar/rays_debug", payload)

    def close(self):
        self._running = False
        self._thread.join(timeout=0.3)
