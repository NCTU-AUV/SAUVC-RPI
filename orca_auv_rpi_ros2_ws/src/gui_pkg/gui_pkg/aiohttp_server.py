import aiohttp
from aiohttp import web
import asyncio
import json
import threading
from pathlib import Path
import gui_pkg

class AIOHTTPServer:
    async def respond_index(request):
        return web.FileResponse(path=Path(gui_pkg.__file__).parent/"index.html")

    def send_topic(self, topic_name, msg):
        payload = json.dumps({"type": "topic", "data": {"topic_name": topic_name, "msg": msg}})

        # Queue messages when no websocket client is connected yet to avoid crashing.
        with self._pending_lock:
            if not self.event_loop or not self.websocket_response or self.websocket_response.closed:
                self._pending_topic_payloads.append(payload)
                return
            loop = self.event_loop
            websocket_response = self.websocket_response

        asyncio.run_coroutine_threadsafe(websocket_response.send_str(payload), loop)

    async def websocket_handler(self, request):
        websocket_response = web.WebSocketResponse(protocols=("protocolOne"))
        await websocket_response.prepare(request)

        self.websocket_response = websocket_response

        # Flush any pending messages accumulated before a client connected.
        pending_payloads = []
        with self._pending_lock:
            pending_payloads, self._pending_topic_payloads = self._pending_topic_payloads, []
        for payload in pending_payloads:
            await websocket_response.send_str(payload)

        async for msg in websocket_response:
            if msg.type == aiohttp.WSMsgType.TEXT:
                if msg.data == 'close':
                    await websocket_response.close()
                else:
                    self._msg_callback(msg.data)
            elif msg.type == aiohttp.WSMsgType.ERROR:
                print('websocket connection closed with exception %s' %
                    websocket_response.exception())

        print('websocket connection closed')

        return websocket_response

    @web.middleware
    async def no_cache_middleware(request, handler):
        response = await handler(request)
        response.headers['Cache-Control'] = 'no-store'
        return response

    def __init__(self, msg_callback):
        self._msg_callback = msg_callback

        app = web.Application(middlewares=[AIOHTTPServer.no_cache_middleware])

        app.router.add_get('/websocket', self.websocket_handler)
        app.router.add_get('/', AIOHTTPServer.respond_index)
        app.router.add_static('/static/', Path(gui_pkg.__file__).parent)

        self.runner = web.AppRunner(app)
        self.websocket_response = None
        self.event_loop = None
        self._pending_topic_payloads = []
        self._pending_lock = threading.Lock()
        self._thread = None
        self._stopped = False

    async def start(self):
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, '0.0.0.0', 80)

        await self.site.start()

    async def stop(self):
        if self._stopped:
            return
        self._stopped = True
        await self.runner.cleanup()

    def run_loop(self):
        self.event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.event_loop)
        self.event_loop.run_until_complete(self.start())
        try:
            self.event_loop.run_forever()
        finally:
            # Ensure cleanup if stop_threading did not already run it.
            self.event_loop.run_until_complete(self.stop())
            self.event_loop.close()

    def start_threading(self):
        self._thread = threading.Thread(target=self.run_loop, daemon=True, name="AIOHTTPServerThread")
        self._thread.start()

    def stop_threading(self, join_timeout: float = 2.0):
        loop = self.event_loop
        if not loop:
            return
        asyncio.run_coroutine_threadsafe(self.stop(), loop)
        loop.call_soon_threadsafe(loop.stop)

        if self._thread:
            self._thread.join(timeout=join_timeout)

if __name__ == "__main__":
    aiohttp_server = AIOHTTPServer(lambda msg: print(msg))
    aiohttp_server.start_threading()

    while True:
        pass
