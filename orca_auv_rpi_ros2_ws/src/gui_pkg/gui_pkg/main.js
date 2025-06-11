let websocket;

if (window.location.hostname == "localhost") {
    websocket = new WebSocket("ws://localhost/websocket", "protocolOne");
} else {
    websocket = new WebSocket("ws://192.168.0.118/websocket", "protocolOne");
}

websocket.onmessage = (event) => {
  console.log(event.data);
};

websocket.onopen = (event) => {
    websocket.send("hello from the frontend!");

    for (let index = 0; index <= 10; index++) {
        websocket.send("count: " + index);
    }
};
