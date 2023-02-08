from CppPythonSocket.server import Server
import datetime
import time


def TimestampMillisec64():
    return int(
        (datetime.datetime.utcnow() - datetime.datetime(1970, 1, 1)).total_seconds()
        * 1000
    )


if __name__ == "__main__":
    server1 = Server("127.0.0.1", 5003)
    # Check that connection works
    ct = 0

    while True:
        start_time = time.time()
        message = server1.receive()
        print(message)
        print("[fps]:", 1.0 / (time.time() - start_time))
