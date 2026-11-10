"""Low-latency H.264 access-unit decoding before the existing GRAY8 odometry."""
from collections import deque
from fractions import Fraction


class H264Decoder:
    def __init__(self):
        self.codec = None
        self.configuration = b""
        self.prepend_configuration = False
        self.headers = deque()
        self.width = self.height = 0

    def configure(self, header, payload):
        try:
            import av
        except ImportError as exc:
            raise RuntimeError("Для H.264 установите зависимости из pc_app/requirements-gui.txt") from exc
        self.close()
        self.codec = av.CodecContext.create("h264", "r")
        self.configuration = bytes(payload)
        self.prepend_configuration = True
        self.width, self.height = header["width"], header["height"]

    def decode(self, header, payload):
        if self.codec is None:
            raise ValueError("H.264 packet arrived before video_config")
        if (header["width"], header["height"]) != (self.width, self.height):
            raise ValueError("H.264 packet geometry differs from video_config")
        import av
        encoded = (self.configuration if self.prepend_configuration else b"") + bytes(payload)
        self.prepend_configuration = False
        packet = av.Packet(encoded)
        # CameraX nanoseconds remain authoritative. Network receive time is never used.
        packet.pts = packet.dts = header["timestamp_ns"]
        packet.time_base = Fraction(1, 1_000_000_000)
        self.headers.append(dict(header))
        decoded = []
        for frame in self.codec.decode(packet):
            if not self.headers:
                raise ValueError("Decoder produced a frame without source metadata")
            source = self.headers.popleft()
            gray = frame.to_ndarray(format="gray")
            if gray.ndim != 2 or gray.shape != (self.height, self.width):
                raise ValueError("Invalid decoded H.264 frame")
            source.update(type="frame", width=int(gray.shape[1]), height=int(gray.shape[0]))
            decoded.append((source, gray.tobytes()))
        return decoded

    def close(self):
        self.codec = None
        self.configuration = b""
        self.prepend_configuration = False
        self.headers.clear()
        self.width = self.height = 0

