from plutolib.logger import Logger
import os


logger = Logger("./logs", "logging")

logger.print("Hello", "Does this work?", comma_seperated=True)
