"""
Flowergirl logging
"""

import logging
import os
import time

try:
    os.mkdir('log')
except FileExistsError:
    pass

formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(name)s - %(message)s')

ch = logging.StreamHandler()
ch.setLevel(logging.INFO)
ch.setFormatter(formatter)

fh = logging.FileHandler(filename="log/flowergirl_{}.log".format(time.strftime("%Y-%m-%d_%H-%M-%S")))
fh.setLevel(logging.DEBUG)
fh.setFormatter(formatter)

if __name__ == "__main__":
    log = logging.getLogger("test")
    log.setLevel(logging.DEBUG)
    log.addHandler(ch)
    log.addHandler(fh)

    log.info("Info message")
    log.debug("Debug message")
    log.warning("Warning message")
    log.error("Error message")
