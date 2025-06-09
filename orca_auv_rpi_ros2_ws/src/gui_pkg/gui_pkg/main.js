const websocket = new WebSocket("ws://localhost/websocket", "protocolOne");

websocket.onmessage = (event) => {
  console.log(event.data);
};

websocket.onopen = (event) => {
    websocket.send("hello from the frontend!");
};
