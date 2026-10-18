"""有界周期直方图和线程安全的窗口统计。"""

from collections import Counter
from math import ceil, sqrt
import threading
import time

BIN_NS = 10_000
MAX_BIN = 1000


class Distribution:
    def __init__(self):
        self.count = self.total = self.squares = 0
        self.minimum = self.maximum = None
        self.bins = Counter()

    def add(self, value):
        value = max(0, int(value))
        self.count += 1
        self.total += value
        self.squares += value*value
        self.minimum = value if self.minimum is None else min(self.minimum, value)
        self.maximum = value if self.maximum is None else max(self.maximum, value)
        self.bins[min((value+BIN_NS-1)//BIN_NS, MAX_BIN+1)] += 1

    def merge(self, row):
        self.count += row["count"]
        self.total += row["sum_ns"]
        self.squares += row["sum_squares_ns2"]
        if row["count"]:
            self.minimum = row["min_ns"] if self.minimum is None else min(self.minimum, row["min_ns"])
            self.maximum = row["max_ns"] if self.maximum is None else max(self.maximum, row["max_ns"])
        self.bins.update({int(k): v for k, v in row["histogram"].items()})

    def snapshot(self):
        targets = [(name, ceil(self.count*fraction)) for name, fraction in
                   (("p50_upper_ns", .5), ("p95_upper_ns", .95),
                    ("p99_upper_ns", .99), ("p999_upper_ns", .999))]
        quantiles = dict.fromkeys(name for name, _ in targets)
        index = seen = 0
        for bucket, count in sorted(self.bins.items()):
            seen += count
            while index < len(targets) and seen >= targets[index][1]:
                quantiles[targets[index][0]] = bucket*BIN_NS if bucket <= MAX_BIN else None
                index += 1
            if index == len(targets):
                break
        mean = self.total/self.count if self.count else None
        return dict(count=self.count, sum_ns=self.total, sum_squares_ns2=self.squares,
                    min_ns=self.minimum, max_ns=self.maximum, mean_ns=mean,
                    stdev_ns=sqrt(max(0, self.squares/self.count-mean*mean)) if self.count else None,
                    histogram=dict(self.bins), histogram_bin_ns=BIN_NS,
                    histogram_max_ns=BIN_NS*MAX_BIN, overflow_count=self.bins[MAX_BIN+1], **quantiles)


class PerformanceProbe:
    def __init__(self):
        self._lock = threading.Lock()
        self._timings = {}
        self._counters = Counter()
        self._peaks = {}
        self._previous = {}
        self._since = time.monotonic_ns()

    def observe(self, name, value):
        with self._lock:
            item = self._timings.get(name)
            if item is None:
                item = self._timings[name] = Distribution()
            item.add(value)

    def tick(self, name, now):
        with self._lock:
            before = self._previous.get(name)
            self._previous[name] = now
            if before is not None:
                item = self._timings.get(name)
                if item is None:
                    item = self._timings[name] = Distribution()
                item.add(now-before)

    def count(self, name, value=1):
        with self._lock:
            self._counters[name] += value

    def peak(self, name, value):
        with self._lock:
            self._peaks[name] = max(value, self._peaks.get(name, 0))

    def reset_clock(self):
        with self._lock:
            self._previous.clear()

    def snapshot(self):
        with self._lock:
            now = time.monotonic_ns()
            start, self._since = self._since, now
            timings, self._timings = self._timings, {}
            counters, self._counters = self._counters, Counter()
            peaks, self._peaks = self._peaks, {}
        return dict(start_ns=start, end_ns=now, duration_s=(now-start)/1e9,
                    timings={name: item.snapshot() for name, item in timings.items()},
                    counters=dict(counters), peaks=peaks)
