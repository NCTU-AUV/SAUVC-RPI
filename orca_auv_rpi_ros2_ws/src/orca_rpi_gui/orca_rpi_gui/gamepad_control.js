/**
 * Xbox Gamepad Controller for AUV Motion Control
 * Uses HTML5 Gamepad API to control the AUV underwater vehicle
 */

// Gamepad configuration
const gamepadConfig = {
    deadzone: 0.15,
    updateIntervalMs: 50,  // 20Hz update rate
    axes: {
        leftStickY: 1,      // Negative = forward (force.x)
        leftStickX: 0,      // Positive = right (force.y)
        rightStickX: 2,     // Positive = right (torque.z)
        rightStickY: 3      // Positive = up (force.z with base_z offset)
    },
    buttons: {
        A: 0,               // Initialize thrusters
        B: 1                // Emergency stop
    }
};

// Gamepad state tracking
let gamepadState = {
    connected: false,
    gamepadIndex: null,
    lastUpdateTime: 0,
    buttonAPressed: false,
    buttonADebounceTime: 500  // ms
};

/**
 * Apply deadzone filtering to analog values
 * @param {number} value - Raw axis value (-1 to 1)
 * @returns {number} Filtered value with deadzone applied
 */
function applyDeadzone(value) {
    if (Math.abs(value) < gamepadConfig.deadzone) {
        return 0;
    }
    // Optional: normalize the value to remove the deadzone gap
    const sign = value > 0 ? 1 : -1;
    return sign * ((Math.abs(value) - gamepadConfig.deadzone) / (1 - gamepadConfig.deadzone));
}

/**
 * Handle gamepad connection
 */
window.addEventListener("gamepadconnected", (event) => {
    console.log("Gamepad connected:", event.gamepad);
    gamepadState.connected = true;
    gamepadState.gamepadIndex = event.gamepad.index;
    gamepadState.lastUpdateTime = Date.now();
    
    // Update UI
    updateGamepadConnectionStatus();
    
    // Start the gamepad polling loop
    requestAnimationFrame(updateGamepadInput);
});

/**
 * Handle gamepad disconnection
 */
window.addEventListener("gamepaddisconnected", (event) => {
    console.log("Gamepad disconnected:", event.gamepad);
    gamepadState.connected = false;
    gamepadState.gamepadIndex = null;
    gamepadState.buttonAPressed = false;
    
    // Update UI
    updateGamepadConnectionStatus();
    
    // Reset all motion control variables to zero
    set_to_zero_simple_motion_control_interface_variale();
});

/**
 * Update gamepad input and control the AUV
 * Called via requestAnimationFrame for smooth polling
 */
function updateGamepadInput() {
    if (!gamepadState.connected) {
        return;
    }
    
    const gamepad = navigator.getGamepads()[gamepadState.gamepadIndex];
    if (!gamepad) {
        gamepadState.connected = false;
        return;
    }
    
    // Check if it's time to send an update (throttle to 20Hz)
    const currentTime = Date.now();
    const shouldUpdate = (currentTime - gamepadState.lastUpdateTime) >= gamepadConfig.updateIntervalMs;
    
    // Process axes (analog sticks) every frame for responsive control
    processGamepadAxes(gamepad);
    
    // Process buttons every frame to catch presses
    processGamepadButtons(gamepad);
    
    // Send update to AUV at the specified interval
    if (shouldUpdate) {
        update_motion_motion_control_wrench();
        gamepadState.lastUpdateTime = currentTime;
    }
    
    // Continue polling
    requestAnimationFrame(updateGamepadInput);
}

/**
 * Process gamepad analog sticks and update motion control
 * @param {Gamepad} gamepad - The gamepad object
 */
function processGamepadAxes(gamepad) {
    if (!gamepad.axes || gamepad.axes.length < 4) {
        return;
    }
    
    // Get axis values with deadzone filtering
    const leftStickY = applyDeadzone(gamepad.axes[gamepadConfig.axes.leftStickY]);
    const leftStickX = applyDeadzone(gamepad.axes[gamepadConfig.axes.leftStickX]);
    const rightStickX = applyDeadzone(gamepad.axes[gamepadConfig.axes.rightStickX]);
    const rightStickY = applyDeadzone(gamepad.axes[gamepadConfig.axes.rightStickY]);
    
    // Get the base z (constant sink force) and thrust limits
    const baseZ = simple_motion_control_interface_variale.constant_sink_force_N;
    const translationalForce = simple_motion_control_interface_variale.translational_push_force_N;
    const rotationalTorque = simple_motion_control_interface_variale.rotational_twist_torque_Nm;
    
    // Map axes to motion control
    // Left stick up/down (Y axis, inverted) -> forward/backward motion (force.x)
    simple_motion_control_interface_variale.current_output_wrench.force.x = 
        -leftStickY * translationalForce;
    
    // Left stick left/right (X axis) -> strafe motion (force.y)
    simple_motion_control_interface_variale.current_output_wrench.force.y = 
        leftStickX * translationalForce;
    
    // Right stick left/right (X axis) -> rotation (torque.z)
    simple_motion_control_interface_variale.current_output_wrench.torque.z = 
        rightStickX * rotationalTorque;
    
    // Right stick up/down (Y axis) -> vertical motion (force.z with base offset)
    simple_motion_control_interface_variale.current_output_wrench.force.z = 
        rightStickY * translationalForce + baseZ;
    
    // Update UI display
    updateGamepadAxisDisplay(leftStickX, leftStickY, rightStickX, rightStickY);
    updateGamepadOutputDisplay();
    
    console.debug("Gamepad axes - X:", simple_motion_control_interface_variale.current_output_wrench.force.x,
                  "Y:", simple_motion_control_interface_variale.current_output_wrench.force.y,
                  "Z:", simple_motion_control_interface_variale.current_output_wrench.force.z,
                  "Tz:", simple_motion_control_interface_variale.current_output_wrench.torque.z);
}

/**
 * Process gamepad buttons and execute corresponding actions
 * @param {Gamepad} gamepad - The gamepad object
 */
function processGamepadButtons(gamepad) {
    if (!gamepad.buttons || gamepad.buttons.length < 2) {
        return;
    }
    
    // Button A (index 0): Initialize thrusters with debounce
    const buttonAPressed = gamepad.buttons[gamepadConfig.buttons.A].pressed;
    const currentTime = Date.now();
    
    if (buttonAPressed && !gamepadState.buttonAPressed) {
        // Button just pressed
        if ((currentTime - gamepadState.lastButtonATime) > gamepadState.buttonADebounceTime) {
            console.log("Button A pressed - Initializing thrusters");
            initialize_all_thrusters_button_onclick();
            gamepadState.lastButtonATime = currentTime;
        }
        gamepadState.buttonAPressed = true;
    } else if (!buttonAPressed && gamepadState.buttonAPressed) {
        // Button released
        gamepadState.buttonAPressed = false;
    }
    
    // Button B (index 1): Emergency stop
    const buttonBPressed = gamepad.buttons[gamepadConfig.buttons.B].pressed;
    
    if (buttonBPressed) {
        console.log("Button B pressed - Emergency stop");
        set_to_zero_simple_motion_control_interface_variale();
    }
    
    // Update button display
    updateGamepadButtonDisplay(buttonAPressed, buttonBPressed);
}

// Initialize the state tracking object
gamepadState.lastButtonATime = 0;

/**
 * Update gamepad connection status display
 */
function updateGamepadConnectionStatus() {
    const statusElement = document.getElementById("gamepad_connection_status");
    if (statusElement) {
        if (gamepadState.connected) {
            statusElement.textContent = "Connected ✓";
            statusElement.style.color = "var(--accent)";
        } else {
            statusElement.textContent = "Not Connected";
            statusElement.style.color = "var(--muted)";
        }
    }
}

/**
 * Update gamepad axis values display
 * @param {number} leftX - Left stick X value
 * @param {number} leftY - Left stick Y value
 * @param {number} rightX - Right stick X value
 * @param {number} rightY - Right stick Y value
 */
function updateGamepadAxisDisplay(leftX, leftY, rightX, rightY) {
    const elements = {
        "gamepad_left_x": Math.round(leftX * 100) / 100,
        "gamepad_left_y": Math.round(leftY * 100) / 100,
        "gamepad_right_x": Math.round(rightX * 100) / 100,
        "gamepad_right_y": Math.round(rightY * 100) / 100
    };
    
    for (const [id, value] of Object.entries(elements)) {
        const elem = document.getElementById(id);
        if (elem) {
            elem.textContent = value.toFixed(2);
        }
    }
}

/**
 * Update gamepad output motion control display
 */
function updateGamepadOutputDisplay() {
    const wrench = simple_motion_control_interface_variale.current_output_wrench;
    
    const elements = {
        "gamepad_output_force_x": Math.round(wrench.force.x * 100) / 100,
        "gamepad_output_force_y": Math.round(wrench.force.y * 100) / 100,
        "gamepad_output_force_z": Math.round(wrench.force.z * 100) / 100,
        "gamepad_output_torque_z": Math.round(wrench.torque.z * 100) / 100
    };
    
    for (const [id, value] of Object.entries(elements)) {
        const elem = document.getElementById(id);
        if (elem) {
            elem.textContent = value.toFixed(2);
        }
    }
}

/**
 * Update button press display
 * @param {boolean} buttonAPressed - A button pressed state
 * @param {boolean} buttonBPressed - B button pressed state
 */
function updateGamepadButtonDisplay(buttonAPressed, buttonBPressed) {
    const buttonAElem = document.getElementById("gamepad_button_a");
    const buttonBElem = document.getElementById("gamepad_button_b");
    
    if (buttonAElem) {
        if (buttonAPressed) {
            buttonAElem.textContent = "Pressed ✓";
            buttonAElem.style.color = "var(--accent)";
        } else {
            buttonAElem.textContent = "Released";
            buttonAElem.style.color = "var(--muted)";
        }
    }
    
    if (buttonBElem) {
        if (buttonBPressed) {
            buttonBElem.textContent = "Pressed ✓";
            buttonBElem.style.color = "#ff6b6b";
        } else {
            buttonBElem.textContent = "Released";
            buttonBElem.style.color = "var(--muted)";
        }
    }
}

console.log("Gamepad control system initialized");
