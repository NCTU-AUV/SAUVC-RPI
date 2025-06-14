const websocket = new WebSocket("ws://" + window.location.hostname + "/websocket", "protocolOne");

websocket.onmessage = (event) => {
  console.log(event.data);

    msg_json_object = JSON.parse(event.data);

    if (msg_json_object.type == "topic") {
        if (msg_json_object.data.topic_name == "is_kill_switch_closed") {
            document.getElementById("is_kill_switch_closed").innerHTML = msg_json_object.data.msg;
        }
    }
};

websocket.onopen = (event) => {
    websocket.send("hello from the frontend!");

    for (let index = 0; index <= 10; index++) {
        websocket.send("count: " + index);
    }
};
