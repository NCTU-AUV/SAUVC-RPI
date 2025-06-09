import aiohttp
from aiohttp import web

async def respond_index(request):
    return web.FileResponse(path="./index.html")

async def respond_script(request):
    return web.FileResponse(path="./main.js")

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

app = web.Application()
app.add_routes([web.get('/', hello),
                web.get('/main.js', respond_script),
                web.get('/websocket', websocket_handler)])

web.run_app(app, port=80)
