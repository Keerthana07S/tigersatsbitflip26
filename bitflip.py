"""Python wrapper for libbitflip.so and example analysis.
Build lib:  g++ -std=c++17 -O2 -shared -fPIC bitflip.cpp capi.cpp -o libbitflip.so
"""
import ctypes, os
import numpy as np

KINDS = ["SEU", "MBU", "STUCK", "SEFI", "READ_GLITCH", "BUS_ERROR"]
STATES = ["clean", "SEU", "MBU", "fault"]

class Event(ctypes.Structure):
    _fields_ = [("scan", ctypes.c_uint64), ("t", ctypes.c_double), ("device", ctypes.c_int32),
                ("addr", ctypes.c_uint32), ("expected", ctypes.c_uint8), ("observed", ctypes.c_uint8),
                ("mask", ctypes.c_uint8), ("kind", ctypes.c_uint8), ("cluster", ctypes.c_int32),
                ("nbytes", ctypes.c_uint32)]

class BitFlipper:
    def __init__(self, dev="/dev/i2c-1", simulate=False, lib="./libbitflip.so"):
        L = self.L = ctypes.CDLL(os.path.abspath(lib))
        L.bf_open.restype = ctypes.c_void_p
        L.bf_open.argtypes = [ctypes.c_char_p, ctypes.c_int]
        for f in ("bf_close", "bf_num_devices", "bf_write_patterns", "bf_scan"):
            getattr(L, f).argtypes = [ctypes.c_void_p]
        L.bf_device_name.restype = ctypes.c_char_p
        L.bf_device_name.argtypes = [ctypes.c_void_p, ctypes.c_int]
        L.bf_get_event.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(Event)]
        L.bf_markov.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_double)]
        L.bf_markov_counts.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_uint64)]
        L.bf_sim_inject.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.h = L.bf_open(dev.encode(), int(simulate))
        if not self.h:
            raise RuntimeError("bf_open failed")
        self.devices = [L.bf_device_name(self.h, i).decode() for i in range(L.bf_num_devices(self.h))]

    def write_patterns(self): return bool(self.L.bf_write_patterns(self.h))
    def inject(self, n): self.L.bf_sim_inject(self.h, n)

    def scan(self):
        n, e, out = self.L.bf_scan(self.h), Event(), []
        for i in range(n):
            self.L.bf_get_event(self.h, i, ctypes.byref(e))
            out.append(dict(scan=e.scan, t=e.t, device=self.devices[e.device], addr=e.addr,
                            expected=e.expected, observed=e.observed, mask=e.mask,
                            kind=KINDS[e.kind], cluster=e.cluster, nbytes=e.nbytes))
        return out

    def markov(self, device):
        P = (ctypes.c_double * 16)()
        self.L.bf_markov(self.h, self.devices.index(device), P)
        return np.array(P).reshape(4, 4)

    def markov_counts(self, device):
        C = (ctypes.c_uint64 * 16)()
        self.L.bf_markov_counts(self.h, self.devices.index(device), C)
        return np.array(C, dtype=float).reshape(4, 4)

    def __del__(self):
        if getattr(self, "h", None):
            self.L.bf_close(self.h); self.h = None


def stationary(P):
    """Long-run fraction of scans spent in each state."""
    w, v = np.linalg.eig(P.T)
    pi = np.real(v[:, np.argmin(abs(w - 1))])
    return pi / pi.sum()

def n_step(P, n):
    """Probability of being in each state n scans from now (fault propagation forecast)."""
    return np.linalg.matrix_power(P, n)


if __name__ == "__main__":
    bfm = BitFlipper(simulate=True)
    bfm.write_patterns()
    for _ in range(50):
        bfm.inject(3)
        bfm.scan()
    for d in bfm.devices:
        P = bfm.markov(d)
        print(f"{d}: stationary {{s: round(float(x), 3) for s, x in zip(STATES, stationary(P))}}")
        print(f"   P(clean -> any upset within 10 scans) = {1 - n_step(P, 10)[0, 0]:.3f}")

    # Some code for analysis of results
    try:
        import pandas as pd
        ev = pd.read_csv("bitflips.csv")
        real = ev[ev.kind.isin(["SEU", "MBU", "STUCK"])]
        print("\nUpsets per device:\n", real.groupby("device").size())
        print("0->1 vs 1->0:", real.dir_0to1.sum(), real.dir_1to0.sum())
        print("Filtered false faults:", (ev.kind == "READ_GLITCH").sum())
    except ImportError:
        pass
