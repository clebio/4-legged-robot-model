from os import environ
import asyncio
from aiohttp import web, WSCloseCode
import aiohttp_jinja2
import jinja2
from os import path

# from settings import config
# from routes import setup_routes
# from middlewares import setup_middlewares


async def handle_greeting(request):
    name = request.match_info.get("name", "Anonymous")
    return web.Response(
        text=f"Hello, {name}.",
        status=200,
    )


async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    async for msg in ws:
        if msg.type == aiohttp.WSMsgType.TEXT:
            if msg.data == "close":
                await ws.close()
            else:
                await ws.send_str(msg.data + "/answer")
        elif msg.type == aiohttp.WSMsgType.ERROR:
            logging.error(f"ws connection closed with exception {ws.exception()}")
    # app.add_routes([web.get('/ws', websocket_handler)])

    return ws


@aiohttp_jinja2.template("index.j2")
async def handle(request):
    return {"title": "Here, now.", "text": f"Quadruped", "app": request.app}


@aiohttp_jinja2.template("controls.j2")
async def controls(request):
    return {"app": request.app}


async def handle_json(request):
    data = {"some": "data"}
    return web.json_response(data)


async def server():
    app = web.Application()
    # setup_routes(app)
    # setup_middlewares(app)
    # app['config'] = config
    aiohttp_jinja2.setup(
        app,
        loader=jinja2.FileSystemLoader(path.join("src", "web", "templates")),
    )

    if environ.get("ENV", "dev").lower() == "dev":
        import aiohttp_debugtoolbar

        aiohttp_debugtoolbar.setup(app)

    routes = [
        web.route("*", "/", handle),
        web.get("/greet/{name}", handle_greeting),
        web.get("/control", controls),
    ]
    app.add_routes(routes)
    # app.add_routes([web.route('*', '/path', all_handler)])

    runner = web.AppRunner(app)
    app.on_shutdown.append(on_shutdown)
    app.on_startup.append(start_background_tasks)
    app.on_cleanup.append(cleanup_background_tasks)

    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", 8080)
    await site.start()
    while True:
        await asyncio.sleep(3600)  # sleep forever by 1 hour intervals


async def on_shutdown(app):
    # runner.cleanup
    for ws in app.get("websockets", []):
        await ws.close(code=WSCloseCode.GOING_AWAY, message="Server shutdown")


async def start_background_tasks(app):
    # app['redis_listener'] = asyncio.create_task(listen_to_redis(app))
    pass


async def cleanup_background_tasks(app):
    for r in app.get("redis_listener", []):
        r.cancel()
        await r
