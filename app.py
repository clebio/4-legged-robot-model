import sys
import asyncio
import logging
from src.web.app import server
from time import time
from functools import partial


async def main(id=0):
    start = time()
    while True:
        loop_time = time() - start
        logging.info(f"Looping {id} at {loop_time:0.2f}")
        await asyncio.sleep(0.1)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logging.getLogger("asyncio").setLevel(logging.WARNING)

    loop = asyncio.new_event_loop()
    tasks = []

    for task in partial(main, 1), server:
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