import aiohttp
from aiohttp import web
import asyncio

class AIOHTTPServer:
    async def respond_index(request):
        return web.FileResponse(path="./index.html")

    async def websocket_handler(request):
        websocket_response = web.WebSocketResponse(protocols=("protocolOne"))
        await websocket_response.prepare(request)

        async for msg in websocket_response:
            if msg.type == aiohttp.WSMsgType.TEXT:
                if msg.data == 'close':
                    await websocket_response.close()
                else:
                    await websocket_response.send_str(msg.data + '/answer')
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

    async def main():
        app = web.Application(middlewares=[AIOHTTPServer.no_cache_middleware])
        app.add_routes([web.get('/', AIOHTTPServer.respond_index),
                web.static('/', "./", show_index=True),
                web.get('/websocket', AIOHTTPServer.websocket_handler)])

        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, 'localhost', 80)
        await site.start()

        while True:
            await asyncio.sleep(3600)  # sleep forever

        await runner.cleanup()

if __name__ == "__main__":
    asyncio.run(AIOHTTPServer.main())
