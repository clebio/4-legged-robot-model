import time
import pybullet as pb


def run(fn, *args, interval=0.05):
    while True:
        now = time.time()
        args = fn(*args)
        pb.stepSimulation()
        time.sleep(interval)