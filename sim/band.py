"""Elastic band (virtual gantry) with a programmatic control channel."""
import os
import socket
import threading

import numpy as np


class Band:
    """Vertical rope-like PD on the torso + weak xy centering.

    Controlled through a unix socket with line commands:
        height <z>   set target torso height (m); enables the band
        release      disable the band
        status       reply "<enabled> <height>"
    """

    def __init__(self, cfg, anchor_xy, height):
        self.cfg = cfg
        self.anchor_xy = np.array(anchor_xy, dtype=float)
        self.height = float(height)
        self.enabled = True
        self._lock = threading.Lock()

    def force(self, pos, vel):
        with self._lock:
            enabled, height = self.enabled, self.height
        if not enabled:
            return np.zeros(3)
        fz = self.cfg.BAND_KZ * (height - pos[2]) - self.cfg.BAND_DZ * vel[2]
        fz = max(fz, 0.0)  # a rope only pulls
        fxy = self.cfg.BAND_KXY * (self.anchor_xy - pos[:2]) - self.cfg.BAND_DXY * vel[:2]
        return np.array([fxy[0], fxy[1], fz])

    # -- control channel ------------------------------------------------
    def handle(self, line):
        parts = line.split()
        with self._lock:
            if parts[:1] == ["height"] and len(parts) == 2:
                self.height = float(parts[1])
                self.enabled = True
                return "ok"
            if parts[:1] == ["release"]:
                self.enabled = False
                return "ok"
            if parts[:1] == ["status"]:
                return f"{int(self.enabled)} {self.height:.4f}"
        return "error"

    def serve(self, path):
        if os.path.exists(path):
            os.unlink(path)
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(path)
        srv.listen(4)

        def loop():
            while True:
                conn, _ = srv.accept()
                with conn, conn.makefile("rw") as f:
                    for line in f:
                        try:
                            reply = self.handle(line)
                        except ValueError:
                            reply = "error"
                        f.write(reply + "\n")
                        f.flush()

        threading.Thread(target=loop, daemon=True, name="band-ctl").start()


class BandClient:
    """Used by the onboard controller (and tests) to drive the band."""

    def __init__(self, path):
        self.path = path
        self._sock = None
        self._f = None

    def _ensure(self):
        if self._f is None:
            self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self._sock.connect(self.path)
            self._f = self._sock.makefile("rw")

    def _ask(self, line):
        for attempt in (0, 1):
            try:
                self._ensure()
                self._f.write(line + "\n")
                self._f.flush()
                return self._f.readline().strip()
            except OSError:
                self.close()
                if attempt:
                    raise

    def set_height(self, z):
        return self._ask(f"height {z:.5f}") == "ok"

    def release(self):
        return self._ask("release") == "ok"

    def status(self):
        enabled, height = self._ask("status").split()
        return bool(int(enabled)), float(height)

    def close(self):
        for o in (self._f, self._sock):
            try:
                o and o.close()
            except OSError:
                pass
        self._f = self._sock = None
