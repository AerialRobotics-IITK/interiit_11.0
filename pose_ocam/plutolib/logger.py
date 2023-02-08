import os
import time


class Logger:
    def __init__(self, log_folder_path: str, prefix: str) -> None:
        os.makedirs(os.path.join(log_folder_path, prefix), exist_ok=True)
        self.LOGFILE = os.path.join(log_folder_path, f"{prefix}/{int(time.time())}.txt")
        self.log_file = open(self.LOGFILE, "w")
        self.did_write = False

    def __del__(self):
        if self.did_write == False:
            print("Log not used, deleting...")
            try:
                os.remove(self.LOGFILE)
            except OSError:
                pass

    def print(self, *args, comma_seperated: bool = False, end: str = "\n", init=False):
        if not init:
            self.did_write = True
        if not comma_seperated:
            print(*args, file=self.log_file, end=end)
        else:
            for arg in args[:-1]:
                print(arg, file=self.log_file, end=",")
            print(args[-1], file=self.log_file, end=end)
