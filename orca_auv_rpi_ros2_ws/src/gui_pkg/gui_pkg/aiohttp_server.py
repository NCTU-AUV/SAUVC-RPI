import aiohttp
from aiohttp import web
import asyncio
import json

class AIOHTTPServer:
    async def respond_index(request):
        return web.FileResponse(path="./index.html")

    async def send_topic(self, topic_name, msg):
        await self.websocket_response.send_str(json.dumps({"type": "topic", "data": {"topic_name": topic_name, "msg": msg}}))

    async def websocket_handler(self, request):
        websocket_response = web.WebSocketResponse(protocols=("protocolOne"))
        await websocket_response.prepare(request)

        self.websocket_response = websocket_response

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
        app.add_routes([web.get('/', AIOHTTPServer.respond_index),
                web.static('/', "./", show_index=True),
                web.get('/websocket', self.websocket_handler)])

        self.runner = web.AppRunner(app)

    async def start(self):
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, 'localhost', 80)

        await self.site.start()

    async def stop(self):
        await self.runner.cleanup()

    def run_loop(self):
        self.event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.event_loop)
        self.event_loop.run_until_complete(self.start())
        self.event_loop.run_forever()

if __name__ == "__main__":
    aiohttp_server = AIOHTTPServer(lambda msg: print(msg))
    aiohttp_server.run_loop()
