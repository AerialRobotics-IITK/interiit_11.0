from threading import Thread
from queue import Queue
import time


class toy:
    def __init__(
        self,
        queue1=None,
        queue2=None,
    ):
        self.q1 = queue1
        self.q2 = queue2

    def publish(self, msg=1):
        self.q2.put(msg)
        print(f"Sent message {msg}")

    def listen(self):
        if self.q1.empty():
            return
        else:
            msg = self.q1.get()
            print(f"Got message {msg}")
    
    def autopilot(self, duration=0.001):
        start = time.time()
        ct = 0
        while time.time() - start < duration:
            ct += 1
            self.publish(msg = ct)
            self.listen()


if __name__ == "__main__":
    q1 = Queue()
    q2 = Queue()
    t1 = Thread(target=toy(q1, q2).autopilot)
    t2 = Thread(target=toy(q2, q1).autopilot)
    t1.start()
    t2.start()