const websocket = new WebSocket("ws://" + window.location.hostname + "/websocket", "protocolOne");

websocket.onmessage = (event) => {
  console.log(event.data);
};

websocket.onopen = (event) => {
    websocket.send("hello from the frontend!");

    for (let index = 0; index <= 10; index++) {
        websocket.send("count: " + index);
    }
};
