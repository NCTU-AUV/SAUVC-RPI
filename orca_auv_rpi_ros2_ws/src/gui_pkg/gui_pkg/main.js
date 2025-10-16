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
    console.log("websocket.onopen");
};

function initialize_all_thrusters_button_onclick(){
      console.log("initialize_all_thrusters_button_onclick");

      websocket.send(JSON.stringify({type: "action", data: {action_name: "initialize_all_thrusters", goal: ""}}));
}

function set_pwm_output_signal_value_us_button_onclick() {
    thruster_number = document.getElementById("thruster_number_input").value;
    set_pwm_output_signal_value_us = document.getElementById("set_pwm_output_signal_value_us_input").value;

    console.log("set_pwm_output_signal_value_us_button_onclick",
                "thruster_number: " + thruster_number,
                "set_pwm_output_signal_value_us: " + set_pwm_output_signal_value_us);

    websocket.send(JSON.stringify({type: "topic", data: {topic_name: "set_pwm_output_signal_value_us", thruster_number: thruster_number, msg: {data: set_pwm_output_signal_value_us}}}));
}

function set_output_wrench_at_center_N_Nm_button_onclick() {
    var msg = {
        force: {
            x: document.getElementById("set_output_wrench_at_center_N_Nm_force_x").value,
            y: document.getElementById("set_output_wrench_at_center_N_Nm_force_y").value,
            z: document.getElementById("set_output_wrench_at_center_N_Nm_force_z").value,
        },
        torque: {
            x: document.getElementById("set_output_wrench_at_center_N_Nm_torque_x").value,
            y: document.getElementById("set_output_wrench_at_center_N_Nm_torque_y").value,
            z: document.getElementById("set_output_wrench_at_center_N_Nm_torque_z").value,
        }
    }

    console.log("set_output_wrench_at_center_N_Nm_button_onclick", msg);

    websocket.send(JSON.stringify({type: "topic", data: {topic_name: "set_output_wrench_at_center_N_Nm", msg: msg}}));
}
