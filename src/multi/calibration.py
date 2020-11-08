import time
import asyncio
import logging
from src.web.app import server
from functools import partial
import concurrent

from src.joystick import Joystick, setup_joystick


INTERVAL = 0.1


async def beeper(id=0):
    start = time()
    while True:
        loop_time = time() - start
        logging.info(f"Beep! {id} at {loop_time:0.2f}")
        await asyncio.sleep(INTERVAL)


async def read_joystick(joystick=False):
    # if not joystick:
    #     joystick = setup_joystick_async()
    #     return joystick
    start = time()
    loop = asyncio.get_running_loop()
    with concurrent.futures.ProcessPoolExecutor() as pool:
        while True:
            loop_time = time() - start
            logging.debug(f"Joystick reader at {loop_time:0.2f}")

            cmd = await loop.run_in_executor(pool, joystick.read)
            if isinstance(oh_joy[0], str) and oh_joy[0] == "Shutdown!":
                logger.info("Shutting down")
                loop.close()
            else:
                logger.info(f"Joystick got {cmd}")
            await asyncio.sleep(INTERVAL)


async def setup_joystick_async():
    loop = asyncio.get_running_loop()
    with concurrent.futures.ProcessPoolExecutor() as pool:
        joystick = await loop.run_in_executor(pool, setup_joystick)
    return joystick


def main():
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)

    loop = asyncio.new_event_loop()

    joystick = asyncio.run(setup_joystick_async())

    tasks = []
    task_list = [
        partial(read_joystick, joystick),
        beeper,
        # server,
    ]
    for task in task_list:
        tasks.append(loop.create_task(task()))

    try:
        loop.run_forever()
    except KeyboardInterrupt as ki:
        pass
    finally:
        for task in tasks:
            task.cancel()
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()
        logging.info("Done with cleanup!")
