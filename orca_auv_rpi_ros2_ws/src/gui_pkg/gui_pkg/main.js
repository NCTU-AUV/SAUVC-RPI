const websocket = new WebSocket("ws://" + window.location.hostname + "/websocket", "protocolOne");

websocket.onmessage = (event) => {
  console.log(event.data);

    msg_json_object = JSON.parse(event.data);

    if (msg_json_object.type == "topic") {
        if (msg_json_object.data.topic_name == "is_kill_switch_closed") {
            document.getElementById("is_kill_switch_closed").innerHTML = msg_json_object.data.msg;
        }
        if (msg_json_object.data.topic_name == "pressure_sensor_depth_m") {
            document.getElementById("pressure_sensor_depth_m").innerHTML = msg_json_object.data.msg;
        }
    }
};

websocket.onopen = (event) => {
    console.log("websocket.onopen");
};

function send_process_action(target, action) {
    websocket.send(JSON.stringify({type: "process", data: {target: target, action: action}}));
}

function send_controller_action(group, action) {
    websocket.send(JSON.stringify({type: "controller", data: {group: group, action: action}}));
}

function enable_bottom_camera_pid_fbc() {
    send_controller_action("bottom_camera_pid_fbc", "enable");
}

function disable_bottom_camera_pid_fbc() {
    send_controller_action("bottom_camera_pid_fbc", "disable");
}

function reset_bottom_camera_pid_fbc() {
    send_controller_action("bottom_camera_pid_fbc", "reset");
}

function enable_depth_control() {
    send_controller_action("depth_control", "enable");
}

function disable_depth_control() {
    send_controller_action("depth_control", "disable");
}

function reset_depth_control() {
    send_controller_action("depth_control", "reset");
}

function set_target_depth_m_button_onclick() {
    const target_depth_m = document.getElementById("target_depth_m_input").value;
    websocket.send(JSON.stringify({type: "topic", data: {topic_name: "set_target_depth_m", msg: {data: target_depth_m}}}));
}

function set_depth_pid_params_button_onclick() {
    const p = document.getElementById("depth_pid_p_input").value;
    const i = document.getElementById("depth_pid_i_input").value;
    const d = document.getElementById("depth_pid_d_input").value;
    const smoothing = document.getElementById("depth_pid_smoothing_input").value;

    websocket.send(JSON.stringify({
        type: "controller",
        data: {
            group: "depth_control",
            action: "set_pid_params",
            params: {
                proportional_gain: p,
                integral_gain: i,
                derivative_gain: d,
                derivative_smoothing_factor: smoothing,
            }
        }
    }));
}

function start_waypoint_target_publisher() {
    send_process_action("waypoint_target_publisher", "start");
}

function stop_waypoint_target_publisher() {
    send_process_action("waypoint_target_publisher", "stop");
}

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
