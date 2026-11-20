"""Isaac Sim -> gz-transport: republishes what isaac/greenhouse_isaac.py streams on the topics the
Gazebo simulation uses, so vio_bridge (OpenVINS), the behaviour tree and the tools run unchanged.

    /<model>/infra1, /<model>/infra2   gz.msgs.Image L_INT8     (D435i infrared pair)
    /<model>/depth                     gz.msgs.Image R_FLOAT32  (D435i depth, m along the optical axis)
    /<model>/picam                     gz.msgs.Image RGB_INT8   (Pi Camera Module 3)
    /<model>/d435i_imu_kin             gz.msgs.IMU              (D435i IMU, computed from the physics)
    /world/<world>/pose/info           gz.msgs.Pose_V           (ground truth, scoring only)
    /world/<world>/clock               gz.msgs.Clock

Runs in .venv-sim (the system Python with the gz bindings); Isaac Sim's own Python cannot load them.

    python -m rowpilot.isaac_relay [--port 9100]
"""

from __future__ import annotations

import argparse
import socket
import time

from .isaac_link import RELAY_PORT, FrameReader

# gz.msgs.Image.PixelFormatType values (as depth_view.py uses them)
PIXEL_FORMAT = {"L8": 1, "RGB8": 3, "F32": 13}
PIXEL_BYTES = {"L8": 1, "RGB8": 3, "F32": 4}


def _stamp_time(time_msg, t: float) -> None:
    ns = int(round(t * 1e9))
    time_msg.sec, time_msg.nsec = ns // 1_000_000_000, ns % 1_000_000_000


def _stamp(header, t: float) -> None:
    _stamp_time(header.stamp, t)


class Publisher:
    def __init__(self) -> None:
        from gz.msgs10.clock_pb2 import Clock
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.imu_pb2 import IMU
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node

        self.types = {"img": Image, "imu": IMU, "pose": Pose_V, "clock": Clock}
        self.node = Node()
        self.pubs: dict[str, object] = {}
        self.counts: dict[str, int] = {}

    def _pub(self, topic: str, kind: str):
        if topic not in self.pubs:
            self.pubs[topic] = self.node.advertise(topic, self.types[kind])
        return self.pubs[topic]

    def publish(self, h: dict, payload: bytes) -> None:
        k, t = h["k"], float(h["t"])
        m = self.types[k]()
        if k == "img":
            _stamp(m.header, t)
            m.width, m.height = h["w"], h["h"]
            m.step = h["w"] * PIXEL_BYTES[h["fmt"]]
            m.pixel_format_type = PIXEL_FORMAT[h["fmt"]]
            m.data = payload
        elif k == "imu":
            _stamp(m.header, t)
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = h["gyro"]
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = h["acc"]
            m.orientation.w = 1.0
        elif k == "pose":
            _stamp(m.header, t)
            for name, x, y, z, qw, qx, qy, qz in h["poses"]:
                p = m.pose.add()
                p.name = name
                _stamp(p.header, t)
                p.position.x, p.position.y, p.position.z = x, y, z
                p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = qw, qx, qy, qz
        elif k == "clock":
            _stamp_time(m.sim, t)
        self._pub(h["topic"], k).publish(m)
        self.counts[h["topic"]] = self.counts.get(h["topic"], 0) + 1


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=RELAY_PORT)
    a = ap.parse_args(argv)
    pub = Publisher()
    srv = socket.create_server(("127.0.0.1", a.port))
    print(f"[isaac_relay] waiting for Isaac Sim on 127.0.0.1:{a.port}", flush=True)
    try:
        serve(srv, pub)
    except KeyboardInterrupt:
        return 0
    return 0


def serve(srv: socket.socket, pub: Publisher) -> None:
    while True:
        conn, _ = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print("[isaac_relay] Isaac Sim connected", flush=True)
        reader, last = FrameReader(), time.monotonic()
        while True:
            data = conn.recv(1 << 20)
            if not data:
                print("[isaac_relay] Isaac Sim disconnected", flush=True)
                break
            for h, payload in reader.feed(data):
                pub.publish(h, payload)
            if time.monotonic() - last > 10:
                last = time.monotonic()
                print("[isaac_relay] " + ", ".join(f"{k}: {v}" for k, v in sorted(pub.counts.items())), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
