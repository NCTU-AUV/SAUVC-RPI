var simple_motion_control_interface_variale = {
    constant_sink_force_N: 0,
    translational_push_force_N: 0,
    rotational_twist_torque_Nm: 0,
    current_output_wrench: {
        force: {
            x: 0,
            y: 0,
            z: 0,
        },
        torque: {
            x: 0,
            y: 0,
            z: 0,
        }
    }
};

function update_motion_motion_control_wrench() {
    var msg = simple_motion_control_interface_variale.current_output_wrench;

    console.log("push_auv", msg);

    websocket.send(JSON.stringify({type: "topic", data: {topic_name: "set_output_wrench_at_center_N_Nm", msg: msg}}));
}

function set_constant_sink_force_button_onclick() {
    simple_motion_control_interface_variale.constant_sink_force_N = Number(document.getElementById("constant_sink_force_N_input").value);
    simple_motion_control_interface_variale.current_output_wrench.force.z = simple_motion_control_interface_variale.constant_sink_force_N;

    update_motion_motion_control_wrench();
}

function set_translational_push_force_button_onclick() {
    simple_motion_control_interface_variale.translational_push_force_N = Number(document.getElementById("translational_push_force_N_input").value);
}

function set_rotational_twist_torque_button_onclick() {
    simple_motion_control_interface_variale.rotational_twist_torque_Nm = Number(document.getElementById("rotational_twist_torque_Nm_input").value);
}

const push_down_button = document.getElementById("push_down_button");

push_down_button.addEventListener("mousedown", () => {
    simple_motion_control_interface_variale.current_output_wrench.force.z =
        simple_motion_control_interface_variale.constant_sink_force_N + simple_motion_control_interface_variale.translational_push_force_N;
    update_motion_motion_control_wrench();
});

push_down_button.addEventListener("mouseup", () => {
    simple_motion_control_interface_variale.current_output_wrench.force.z = simple_motion_control_interface_variale.constant_sink_force_N;
    update_motion_motion_control_wrench();
});

const push_up_button = document.getElementById("push_up_button");

push_up_button.addEventListener("mousedown", () => {
    simple_motion_control_interface_variale.current_output_wrench.force.z =
        simple_motion_control_interface_variale.constant_sink_force_N - simple_motion_control_interface_variale.translational_push_force_N;
    update_motion_motion_control_wrench();
});

push_up_button.addEventListener("mouseup", () => {
    simple_motion_control_interface_variale.current_output_wrench.force.z = simple_motion_control_interface_variale.constant_sink_force_N;
    update_motion_motion_control_wrench();
});

const push_left_button = document.getElementById("push_left_button");

push_left_button.addEventListener("mousedown", () => {
    simple_motion_control_interface_variale.current_output_wrench.force.y = - simple_motion_control_interface_variale.translational_push_force_N;
    update_motion_motion_control_wrench();
});

push_left_button.addEventListener("mouseup", () => {
    simple_motion_control_interface_variale.current_output_wrench.force.y = 0;
    update_motion_motion_control_wrench();
});

const push_right_button = document.getElementById("push_right_button");

push_right_button.addEventListener("mousedown", () => {
    simple_motion_control_interface_variale.current_output_wrench.force.y = simple_motion_control_interface_variale.translational_push_force_N;
    update_motion_motion_control_wrench();
});

push_right_button.addEventListener("mouseup", () => {
    simple_motion_control_interface_variale.current_output_wrench.force.y = 0;
    update_motion_motion_control_wrench();
});

const push_backward_button = document.getElementById("push_backward_button");

push_backward_button.addEventListener("mousedown", () => {
    simple_motion_control_interface_variale.current_output_wrench.force.x = - simple_motion_control_interface_variale.translational_push_force_N;
    update_motion_motion_control_wrench();
});

push_backward_button.addEventListener("mouseup", () => {
    simple_motion_control_interface_variale.current_output_wrench.force.x = 0;
    update_motion_motion_control_wrench();
});

const push_forward_button = document.getElementById("push_forward_button");

push_forward_button.addEventListener("mousedown", () => {
    simple_motion_control_interface_variale.current_output_wrench.force.x = simple_motion_control_interface_variale.translational_push_force_N;
    update_motion_motion_control_wrench();
});

push_forward_button.addEventListener("mouseup", () => {
    simple_motion_control_interface_variale.current_output_wrench.force.x = 0;
    update_motion_motion_control_wrench();
});

const twist_rightward_button = document.getElementById("twist_rightward_button");

twist_rightward_button.addEventListener("mousedown", () => {
    simple_motion_control_interface_variale.current_output_wrench.torque.z = simple_motion_control_interface_variale.rotational_twist_torque_Nm;
    update_motion_motion_control_wrench();
});

twist_rightward_button.addEventListener("mouseup", () => {
    simple_motion_control_interface_variale.current_output_wrench.torque.z = 0;
    update_motion_motion_control_wrench();
});

const twist_leftward_button = document.getElementById("twist_leftward_button");

twist_leftward_button.addEventListener("mousedown", () => {
    simple_motion_control_interface_variale.current_output_wrench.torque.z = - simple_motion_control_interface_variale.rotational_twist_torque_Nm;
    update_motion_motion_control_wrench();
});

twist_leftward_button.addEventListener("mouseup", () => {
    simple_motion_control_interface_variale.current_output_wrench.torque.z = 0;
    update_motion_motion_control_wrench();
});

document.addEventListener("keydown", (event) => {
    switch (event.key) {
        case "w":
            push_forward_button.dispatchEvent(new Event("mousedown"));
            break;
        case "s":
            push_backward_button.dispatchEvent(new Event("mousedown"));
            break;
        case "a":
            push_left_button.dispatchEvent(new Event("mousedown"));
            break;
        case "d":
            push_right_button.dispatchEvent(new Event("mousedown"));
            break;
        case "q":
            twist_leftward_button.dispatchEvent(new Event("mousedown"));
            break;
        case "e":
            twist_rightward_button.dispatchEvent(new Event("mousedown"));
            break;
        case "ArrowUp":
            push_up_button.dispatchEvent(new Event("mousedown"));
            break;
        case "ArrowDown":
            push_down_button.dispatchEvent(new Event("mousedown"));
            break;
    }
});

document.addEventListener("keyup", (event) => {
    switch (event.key) {
        case "w":
            push_forward_button.dispatchEvent(new Event("mouseup"));
            break;
        case "s":
            push_backward_button.dispatchEvent(new Event("mouseup"));
            break;
        case "a":
            push_left_button.dispatchEvent(new Event("mouseup"));
            break;
        case "d":
            push_right_button.dispatchEvent(new Event("mouseup"));
            break;
        case "q":
            twist_leftward_button.dispatchEvent(new Event("mouseup"));
            break;
        case "e":
            twist_rightward_button.dispatchEvent(new Event("mouseup"));
            break;
        case "ArrowUp":
            push_up_button.dispatchEvent(new Event("mouseup"));
            break;
        case "ArrowDown":
            push_down_button.dispatchEvent(new Event("mouseup"));
            break;
    }
});
