"""
Flowergirl logging
"""

import logging
import os
import time

__all__ = ["handler"]

try:
    os.mkdir('log')
except FileExistsError:
    pass
logging.basicConfig(filename="log/flowergirl_{}.log".format(time.strftime("%Y-%m-%d_%H-%M-%S")))

formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(name)s - %(message)s')

handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
handler.setFormatter(formatter)

if __name__ == "__main__":
    log = logging.getLogger("test")
    log.addHandler(handler)

    log.info("Info message")
    log.debug("Debug message")
    log.warning("Warning message")
    log.error("Error message")
