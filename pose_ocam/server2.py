from CppPythonSocket.server import Server
import datetime
import time
from plutolib.utils import Filter


def TimestampMillisec64():
    return int(
        (datetime.datetime.utcnow() - datetime.datetime(1970, 1, 1)).total_seconds()
        * 1000
    )


if __name__ == "__main__":
    server1 = Server("127.0.0.1", 8000)
    # Check that connection works
    fps_estimate = Filter(r=3)

    while True:
        start_time = time.time()
        message = server1.receive()
        print(message)
        fps = 1.0 / (time.time() - start_time)
        fps = fps_estimate.predict_kalman(fps)
        print("[fps]:", fps)
