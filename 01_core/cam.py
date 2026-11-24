import cv2
import numpy as np
import config as C


class Cam:
    def __init__(self):
        from picamera2 import Picamera2
        self.pc = Picamera2()
        cfg = self.pc.create_preview_configuration(
            main={"size": (C.CAM_W, C.CAM_H), "format": "RGB888"}
        )
        self.pc.configure(cfg)
        self.pc.start()
        self.lock_exposure()

    def lock_exposure(self, settle=1.2):
        import time
        time.sleep(settle)
        try:
            md = self.pc.capture_metadata()
            exp = int(md["ExposureTime"])
            gain = float(md["AnalogueGain"])


            if exp > C.MAX_EXPOSURE_US:
                gain = min(gain * (exp / C.MAX_EXPOSURE_US), C.MAX_GAIN)
                exp = C.MAX_EXPOSURE_US
            self.pc.set_controls({
                "AeEnable": False,
                "ExposureTime": exp,
                "AnalogueGain": gain,
                "AwbEnable": False,
                "ColourGains": tuple(md["ColourGains"]),
            })
            print(f"  exposure locked at {exp/1000:.0f} ms, gain {gain:.1f} "
                  f"-> up to {1e6/exp:.0f} fps", flush=True)
            time.sleep(0.3)
        except Exception as e:
            print(f"  (could not lock exposure: {e})")
    ROT = {90: cv2.ROTATE_90_CLOCKWISE,
           180: cv2.ROTATE_180,
           270: cv2.ROTATE_90_COUNTERCLOCKWISE}

    def frame(self):
        f = self.pc.capture_array()
        if C.SWAP_RB:
            f = f[:, :, ::-1].copy()
        if C.ROTATE_DEG in self.ROT:
            f = cv2.rotate(f, self.ROT[C.ROTATE_DEG])
        if C.FLIP:
            f = cv2.rotate(f, cv2.ROTATE_180)
        return f

    def close(self):
        try:
            self.pc.stop()
        except Exception:
            pass
