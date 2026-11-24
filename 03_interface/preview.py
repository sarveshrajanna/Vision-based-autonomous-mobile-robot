import time
import config as C
import stream
from cam import Cam
cam = Cam()
stream.start(C.STREAM_PORT)
print(f"streaming on port {C.STREAM_PORT} - open http://<pi-ip>:{C.STREAM_PORT}/", flush=True)
try:
    while True:
        stream.publish(cam.frame())
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    cam.close()
