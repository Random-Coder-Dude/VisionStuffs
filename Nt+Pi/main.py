#!/usr/bin/env python3

import struct
import time
import os
import threading
from datetime import datetime

import ntcore

TEAM = 1403
USB_PATH = "/media/usb/nt_logs"

NT_TYPE_MAP = {
    "boolean":   "boolean",
    "double":    "double",
    "float":     "float",
    "int":       "int64",
    "string":    "string",
    "json":      "string",
    "raw":       "raw",
    "boolean[]": "boolean[]",
    "double[]":  "double[]",
    "float[]":   "float[]",
    "int[]":     "int64[]",
    "string[]":  "string[]",
}


def now_us():
    return int(time.time() * 1_000_000)


class WPILogWriter:

    HEADER_MAGIC = b"WPILOG"
    VERSION = 0x0100

    def __init__(self, path):
        self.f = open(path, "wb")
        self.lock = threading.Lock()
        self.next_id = 1
        self.entries = {}

        self.f.write(self.HEADER_MAGIC)
        self.f.write(struct.pack("<H", self.VERSION))
        self.f.write(struct.pack("<I", 0))  # no extra header

    def _encode(self, val, max_bytes):
        for n in range(1, max_bytes + 1):
            if val < (1 << (8 * n)):
                return val.to_bytes(n, "little")
        return val.to_bytes(max_bytes, "little")

    def _write_record(self, entry, payload, ts):
        eid = self._encode(entry, 4)
        pay = self._encode(len(payload), 4)
        tsb = self._encode(max(ts, 0), 8)

        bit = (len(eid) - 1) | ((len(pay) - 1) << 2) | ((len(tsb) - 1) << 4)

        with self.lock:
            self.f.write(bytes([bit]))
            self.f.write(eid)
            self.f.write(pay)
            self.f.write(tsb)
            self.f.write(payload)

    def start(self, name, type_str):
        entry = self.next_id
        self.next_id += 1
        self.entries[name] = entry

        name_b = name.encode()
        type_b = type_str.encode()

        payload = (
            bytes([0]) +
            struct.pack("<I", entry) +
            struct.pack("<I", len(name_b)) + name_b +
            struct.pack("<I", len(type_b)) + type_b +
            struct.pack("<I", 0)  # no metadata
        )

        self._write_record(0, payload, now_us())
        return entry

    def finish(self, entry_id, ts):
        payload = bytes([1]) + struct.pack("<I", entry_id)
        self._write_record(0, payload, ts)

    def append(self, entry, payload, ts):
        self._write_record(entry, payload, ts)

    def close(self):
        self.f.flush()
        self.f.close()


def main():
    os.makedirs(USB_PATH, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = f"{USB_PATH}/nt_{stamp}.wpilog"

    print("Logging to:", path)

    writer = WPILogWriter(path)

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("nt_logger")
    inst.setServerTeam(TEAM)

    tracked = {}  # topic name -> entry id
    subs = {}     # topic name -> subscriber (kept alive to avoid GC)

    poller = ntcore.NetworkTableListenerPoller(inst)
    poller.addListener(
        ["/"],
        ntcore.EventFlags.kTopic |
        ntcore.EventFlags.kValueAll |
        ntcore.EventFlags.kImmediate,
    )

    print("Logging... Press Ctrl+C to stop.\n")

    try:
        while True:
            events = poller.readQueue()

            for event in events:

                # --- New topic announced ---
                if event.flags & ntcore.EventFlags.kTopic:
                    if event.flags & ntcore.EventFlags.kPublish:
                        name = event.data.name
                        nt_type = event.data.typeStr

                        wpilog = NT_TYPE_MAP.get(nt_type)
                        if wpilog is None:
                            print(f"  [skip] unsupported type '{nt_type}' for {name}")
                            continue

                        entry = writer.start(name, wpilog)
                        tracked[name] = entry

                        # Subscribe and store reference so it isn't garbage collected
                        topic = inst.getTopic(name)
                        subs[name] = topic.genericSubscribe(ntcore.PubSubOptions())

                        print(f"  [+] {name}  ({nt_type} -> {wpilog})")

                # --- Value update ---
                if event.flags & ntcore.EventFlags.kValueAll:
                    val = event.data
                    name = val.topic.getName()

                    entry = tracked.get(name)
                    if entry is None:
                        continue

                    v = val.value
                    ts = val.time if val.time else now_us()

                    try:
                        if isinstance(v, bool):
                            payload = bytes([1 if v else 0])

                        elif isinstance(v, int):
                            payload = struct.pack("<q", v)

                        elif isinstance(v, float):
                            payload = struct.pack("<d", v)

                        elif isinstance(v, str):
                            payload = v.encode()

                        elif isinstance(v, (bytes, bytearray)):
                            payload = bytes(v)

                        elif isinstance(v, list) and len(v) > 0:
                            # Check bool before int — bool is subclass of int in Python
                            if isinstance(v[0], bool):
                                payload = bytes([1 if x else 0 for x in v])
                            elif isinstance(v[0], int):
                                payload = struct.pack(f"<{len(v)}q", *v)
                            elif isinstance(v[0], float):
                                payload = struct.pack(f"<{len(v)}d", *v)
                            elif isinstance(v[0], str):
                                encoded = [s.encode() for s in v]
                                payload = struct.pack("<I", len(encoded))
                                for s in encoded:
                                    payload += struct.pack("<I", len(s)) + s
                            else:
                                payload = str(v).encode()

                        elif isinstance(v, list):
                            continue  # empty list, nothing to write

                        else:
                            payload = str(v).encode()

                        writer.append(entry, payload, ts)

                    except Exception as e:
                        print(f"  [warn] failed to encode {name}: {e}")

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        ts = now_us()
        for name, entry_id in tracked.items():
            writer.finish(entry_id, ts)
        writer.close()
        inst.stopClient()
        print(f"Saved: {path}")


if __name__ == "__main__":
    main()