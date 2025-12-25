import asyncio
import aiohttp
import json
import time

async def test_client():
    url = 'http://localhost:8000/websocket'
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(url) as ws:
            print("Connected to WebSocket")
            
            # Message simulating the frontend joystick command
            msg = {
                "type": "topic",
                "data": {
                    "topic_name": "set_output_wrench_at_center_N_Nm",
                    "msg": {
                        "force": {"x": 10.0, "y": 0.0, "z": 0.0},
                        "torque": {"x": 0.0, "y": 0.0, "z": 5.0}
                    }
                }
            }
            
            print(f"Sending message: {json.dumps(msg)}")
            await ws.send_json(msg)
            
            # Wait a bit to ensure it's processed
            await asyncio.sleep(1)
            print("Message sent, closing connection")
            await ws.close()

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(test_client())
