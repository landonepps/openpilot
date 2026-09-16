"""Use the rlog clock domain, including time spent suspended on Linux devices."""
import time

LOG_CLOCK = getattr(time, 'CLOCK_BOOTTIME', time.CLOCK_MONOTONIC)
LOG_CLOCK_NAME = 'CLOCK_BOOTTIME' if hasattr(time, 'CLOCK_BOOTTIME') else 'CLOCK_MONOTONIC'


def log_time_ns():
  return time.clock_gettime_ns(LOG_CLOCK)
