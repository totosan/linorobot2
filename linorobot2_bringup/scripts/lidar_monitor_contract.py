"""ROS-independent wall-clock health contract for the required lidar scan."""


class ScanHealth:
    """Detect failure to start or a stale scan stream using monotonic time."""

    def __init__(self, start_time, startup_grace_period=10.0, scan_timeout=15.0):
        if startup_grace_period <= 0 or scan_timeout <= 0:
            raise ValueError("scan grace period and timeout must be positive")
        self.start_time = float(start_time)
        self.startup_grace_period = float(startup_grace_period)
        self.scan_timeout = float(scan_timeout)
        self.last_scan_time = None

    def observe_scan(self, now):
        self.last_scan_time = float(now)

    def failure_reason(self, now):
        now = float(now)
        if self.last_scan_time is None:
            if now - self.start_time >= self.startup_grace_period:
                return (
                    "No scan data received after "
                    f"{self.startup_grace_period:.1f}s startup period"
                )
            return None

        if now - self.last_scan_time > self.scan_timeout:
            return f"No scan data received for more than {self.scan_timeout:.1f}s"
        return None
