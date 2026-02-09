const websocket = new WebSocket("ws://" + window.location.hostname + "/websocket", "protocolOne");
const stm32LogState = {
    shouldAutoScroll: true,
};

function isAtBottom(element, thresholdPx = 8) {
    return element.scrollTop + element.clientHeight >= element.scrollHeight - thresholdPx;
}

function updateStm32LogAutoScrollState() {
    const element = document.getElementById("stm32_debug_log");
    if (!element) {
        return;
    }
    stm32LogState.shouldAutoScroll = isAtBottom(element);
}

document.addEventListener("DOMContentLoaded", () => {
    const element = document.getElementById("stm32_debug_log");
    if (!element) {
        return;
    }
    element.addEventListener("scroll", updateStm32LogAutoScrollState);
    stm32LogState.shouldAutoScroll = isAtBottom(element);
});

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
        if (msg_json_object.data.topic_name == "set_pwm_output_signal_value_us") {
            const pwmValues = msg_json_object.data.msg || [];
            for (let i = 0; i < 8; i += 1) {
                const value = pwmValues[i] ?? "";
                const element = document.getElementById("pwm_output_signal_value_us_" + i);
                if (element) {
                    element.innerHTML = value;
                }
            }
        }
        if (msg_json_object.data.topic_name == "flash_stm32_status") {
            const status = msg_json_object.data.msg || {};
            const message = status.message || "";
            const successText = status.success === true ? "success" : "failed";
            const element = document.getElementById("flash_stm32_status");
            if (element) {
                element.innerHTML = message ? `${successText}: ${message}` : successText;
            }
        }
        if (msg_json_object.data.topic_name == "stm32_debug_log") {
            const element = document.getElementById("stm32_debug_log");
            if (element) {
                const line = msg_json_object.data.msg || "";
                if (line) {
                    const lines = element.textContent.split("\n").filter(Boolean);
                    lines.push(line);
                    const maxLines = 200;
                    const trimmed = lines.slice(-maxLines);
                    element.textContent = trimmed.join("\n");
                    if (stm32LogState.shouldAutoScroll) {
                        element.scrollTop = element.scrollHeight;
                    }
                }
            }
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

function flash_stm32_button_onclick() {
    console.log("flash_stm32_button_onclick");
    const element = document.getElementById("flash_stm32_status");
    if (element) {
        element.innerHTML = "running...";
    }
    const logElement = document.getElementById("stm32_debug_log");
    if (logElement) {
        logElement.textContent = "";
        logElement.scrollTop = logElement.scrollHeight;
        stm32LogState.shouldAutoScroll = true;
    }
    websocket.send(JSON.stringify({type: "action", data: {action_name: "flash_stm32"}}));
}

function clear_stm32_log_button_onclick() {
    const logElement = document.getElementById("stm32_debug_log");
    if (logElement) {
        logElement.textContent = "";
        logElement.scrollTop = logElement.scrollHeight;
        stm32LogState.shouldAutoScroll = true;
    }
}

function set_pwm_output_signal_value_us_button_onclick() {
    const pwm_values = [];
    for (let i = 0; i < 8; i += 1) {
        const value = document.getElementById("set_pwm_output_signal_value_us_" + i).value;
        pwm_values.push(value);
    }

    console.log("set_pwm_output_signal_value_us_button_onclick", pwm_values);

    websocket.send(JSON.stringify({
        type: "topic",
        data: {
            topic_name: "set_pwm_output_signal_value_us",
            msg: {data: pwm_values}
        }
    }));
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
