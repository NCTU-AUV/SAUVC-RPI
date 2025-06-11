import aiohttp
from aiohttp import web

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

app = web.Application(middlewares=[no_cache_middleware])
app.add_routes([web.get('/', respond_index),
                web.static('/', "./", show_index=True),
                web.get('/websocket', websocket_handler)])

web.run_app(app, port=80)
