import asyncio
import websockets
import json

async def mock_handler(websocket):
    print(f"Client connected: {websocket.remote_address}")
    try:
        # Send initial status
        await websocket.send(json.dumps({
            "type": "topic",
            "data": {
                "topic_name": "is_kill_switch_closed",
                "msg": False
            }
        }))
        
        async for message in websocket:
            try:
                data = json.loads(message)
                if data.get("type") == "topic" and data.get("data", {}).get("topic_name") == "set_output_wrench_at_center_N_Nm":
                    wrench = data["data"]["msg"]
                    force = wrench["force"]
                    torque = wrench["torque"]
                    print(f"Received Wrench: F:[{force['x']:.2f}, {force['y']:.2f}, {force['z']:.2f}] T_z:{torque['z']:.2f}")
                else:
                    print(f"Received message: {message}")
            except json.JSONDecodeError:
                print(f"Non-JSON message: {message}")
    except websockets.exceptions.ConnectionClosedOK:
        print("Client disconnected normally")
    except Exception as e:
        print(f"Error: {e}")

async def main():
    print("Starting Mock ROS WebSocket Server on ws://localhost:80/websocket")
    async with websockets.serve(mock_handler, "localhost", 80, path="/websocket"):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer stopped.")
