const wsStatusElement = document.getElementById("ws_status");
const gamepadStatusElement = document.getElementById("gamepad_status");
const thrustXElement = document.getElementById("thrust_x");
const thrustYElement = document.getElementById("thrust_y");
const thrustZElement = document.getElementById("thrust_z");
const thrustYawElement = document.getElementById("thrust_yaw");
const cameraFeedElement = document.getElementById("camera_feed");

const max_push = 10;
const max_twist = 5;
const deadzone = 0.15;
const sendIntervalMs = 50; // 20 Hz

const current_wrench = {
    force: {
        x: 0,
        y: 0,
        z: 0,
    },
    torque: {
        x: 0,
        y: 0,
        z: 0,
    },
};

let activeGamepadIndex = null;
let animationFrameId = null;
let lastSendTime = 0;
let lastBPressed = false;

const websocket = new WebSocket("ws://" + window.location.hostname + "/websocket", "protocolOne");

function initializeCameraFeed() {
    if (!cameraFeedElement) {
        return;
    }

    const bottomCameraTopic = "/orca_auv/bottom_camera/image_raw";
    const streamUrl = "http://" + window.location.hostname + ":8080/stream?topic="
        + encodeURIComponent(bottomCameraTopic)
        + "&type=ros_compressed";
    cameraFeedElement.src = streamUrl;
}

function updateWsStatus(connected) {
    if (!wsStatusElement) {
        return;
    }
    wsStatusElement.textContent = connected ? "🟢 Connected" : "🔴 Disconnected";
}

function updateGamepadStatus(text) {
    if (!gamepadStatusElement) {
        return;
    }
    gamepadStatusElement.textContent = text;
}

function updateHudWrench() {
    if (thrustXElement) {
        thrustXElement.textContent = current_wrench.force.x.toFixed(2);
    }
    if (thrustYElement) {
        thrustYElement.textContent = current_wrench.force.y.toFixed(2);
    }
    if (thrustZElement) {
        thrustZElement.textContent = current_wrench.force.z.toFixed(2);
    }
    if (thrustYawElement) {
        thrustYawElement.textContent = current_wrench.torque.z.toFixed(2);
    }
}

function applyDeadzone(value) {
    if (Math.abs(value) < deadzone) {
        return 0;
    }
    return value;
}

function buildWrenchPayload() {
    return JSON.stringify({
        type: "topic",
        data: {
            topic_name: "set_output_wrench_at_center_N_Nm",
            msg: current_wrench,
        },
    });
}

function sendCurrentWrench() {
    if (websocket.readyState !== WebSocket.OPEN) {
        return;
    }
    websocket.send(buildWrenchPayload());
}

function setWrenchToZero() {
    current_wrench.force.x = 0;
    current_wrench.force.y = 0;
    current_wrench.force.z = 0;
    current_wrench.torque.x = 0;
    current_wrench.torque.y = 0;
    current_wrench.torque.z = 0;
    updateHudWrench();
}

function tickGamepad() {
    const gamepads = navigator.getGamepads();
    const gamepad = activeGamepadIndex !== null ? gamepads[activeGamepadIndex] : null;

    if (!gamepad) {
        animationFrameId = requestAnimationFrame(tickGamepad);
        return;
    }

    const leftX = applyDeadzone(gamepad.axes[0] || 0);
    const leftY = applyDeadzone(gamepad.axes[1] || 0);
    const rightX = applyDeadzone(gamepad.axes[2] || 0);
    const rightY = applyDeadzone(gamepad.axes[3] || 0);

    // Left stick up/down controls surge on force.x (up -> positive x)
    current_wrench.force.x = -leftY * max_push;
    // Left stick left/right controls sway on force.y
    current_wrench.force.y = leftX * max_push;
    // Right stick up/down controls heave on force.z
    current_wrench.force.z = -rightY * max_push;
    // Right stick left/right controls yaw on torque.z
    current_wrench.torque.z = rightX * max_twist;

    updateHudWrench();

    const bPressed = Boolean(gamepad.buttons[1] && gamepad.buttons[1].pressed);
    if (bPressed && !lastBPressed) {
        setWrenchToZero();
        sendCurrentWrench();
    }
    lastBPressed = bPressed;

    const now = performance.now();
    if (now - lastSendTime >= sendIntervalMs) {
        sendCurrentWrench();
        lastSendTime = now;
    }

    animationFrameId = requestAnimationFrame(tickGamepad);
}

window.addEventListener("gamepadconnected", (event) => {
    activeGamepadIndex = event.gamepad.index;
    updateGamepadStatus("🎮 Connected: " + event.gamepad.id);

    if (animationFrameId === null) {
        animationFrameId = requestAnimationFrame(tickGamepad);
    }
});

window.addEventListener("gamepaddisconnected", (event) => {
    if (activeGamepadIndex === event.gamepad.index) {
        activeGamepadIndex = null;
        lastBPressed = false;
        updateGamepadStatus("🎮 請按下搖桿任意鍵...");
        setWrenchToZero();
        sendCurrentWrench();
    }
});

websocket.onopen = () => {
    updateWsStatus(true);
};

websocket.onclose = () => {
    updateWsStatus(false);
};

websocket.onerror = () => {
    updateWsStatus(false);
};

updateWsStatus(false);
updateHudWrench();
initializeCameraFeed();
