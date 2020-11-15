import time
import pybullet as pb
import sys
import logging as _logging

logging = _logging.getLogger(__name__)
logging.setLevel(_logging.INFO)


def run(fn, *args, interval=0.02):
    while True:
        now = time.time()
        args = fn(*args)
        pb.stepSimulation()
        time.sleep(interval)


def reset_servos():
    from src.arduino import get_ACM

    arduino = get_ACM()
    if not arduino:
        logging.error("No Arduino connected")
        sys.exit(1)
    arduino.clear()
    arduino.send("<QUIT>")
    logger.info("Disabling relay")
    arduino.send("<RELAY#off>")
