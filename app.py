import sys
import asyncio
import logging
from base64 import b64encode
from src.web.app import server
from time import time
from functools import partial


def io_bound(len=100):
    # File operations (such as logging) can block the
    # event loop: run them in a thread pool.
    with open("/dev/urandom", "rb") as f:
        return b64encode(f.read(len)).decode("utf-8")


def cpu_bound(p=5):
    # CPU-bound operations will block the event loop:
    # in general it is preferable to run them in a
    # process pool.
    return sum(i * i for i in range(10 ** p))


async def looper(id=0):
    start = time()
    while True:
        loop_time = time() - start
        logging.info(f"Looping {id} at {loop_time:0.2f}")
        await asyncio.sleep(0.1)


async def blocking_io(*args, **kwargs):
    loop = asyncio.get_running_loop()
    with concurrent.futures.ThreadPoolExecutor() as pool:
        val = await loop.run_in_executor(pool, partial(io_bound, *args, **kwargs))
        logging.info(f"custom thread pool {val}")


async def blocking_cpu(*args, **kwargs):
    loop = asyncio.get_running_loop()
    with concurrent.futures.ProcessPoolExecutor() as pool:
        val = await loop.run_in_executor(pool, partial(cpu_bound, *args, **kwargs))
        logging.info(f"custom process pool {val}")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logging.getLogger("asyncio").setLevel(logging.WARNING)

    loop = asyncio.new_event_loop()
    tasks = []

    task_list = [
        # partial(looper, 1),
        # partial(blocking_cpu, p=7),
        # partial(blocking_io, len=100),
        server,
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
