const GuiProtocol = Object.freeze({
    websocketPath: "/websocket",
    websocketSubprotocol: "protocolOne",
    fields: Object.freeze({
        type: "type",
        data: "data",
        topicName: "topic_name",
        msg: "msg",
        actionName: "action_name",
        target: "target",
        action: "action",
        group: "group",
        params: "params",
    }),
    types: Object.freeze({
        action: "action",
        topic: "topic",
        process: "process",
        controller: "controller",
    }),
    actions: Object.freeze({
        initializeAllThrusters: "initialize_all_thrusters",
        flashStm32: "flash_stm32",
        setSupervisorSimulationMode: "set_supervisor_simulation_mode",
    }),
    processTargets: Object.freeze({
        waypointTargetPublisher: "waypoint_target_publisher",
    }),
    processActions: Object.freeze({
        start: "start",
        stop: "stop",
        kill: "kill",
    }),
    controllerGroups: Object.freeze({
        bottomCameraPidFbc: "bottom_camera_pid_fbc",
        depthControl: "depth_control",
    }),
    controllerActions: Object.freeze({
        enable: "enable",
        disable: "disable",
        reset: "reset",
        setPidParams: "set_pid_params",
    }),
    topics: Object.freeze({
        killSwitchClosed: "sensors/kill_switch_closed",
        depthM: "sensors/depth_m",
        stm32Log: "diagnostics/stm32/log",
        systemManagerMode: "system_manager/mode",
        systemManagerStatus: "system_manager/status",
        bottomCameraPidXReferencePx: "control/pid/bottom_camera/x/reference_px",
        bottomCameraPidYReferencePx: "control/pid/bottom_camera/y/reference_px",
        bottomCameraPidXFeedbackPx: "control/pid/bottom_camera/x/feedback_px",
        bottomCameraPidYFeedbackPx: "control/pid/bottom_camera/y/feedback_px",
        thrustersPwmUs: "thrusters/pwm_us",
        thrustersEnabled: "thrusters/enabled",
        electromagnetEnabled: "actuators/electromagnet/enabled",
        wrenchCommand: "control/wrench_command",
        targetDepthM: "control/targets/depth_m",
        flashStm32Status: "flash_stm32_status",
    }),
    makeWebsocketUrl(hostname) {
        return "ws://" + hostname + this.websocketPath;
    },
    makeTopicMessage(topicName, msg) {
        return {
            type: this.types.topic,
            data: {
                topic_name: topicName,
                msg: msg,
            },
        };
    },
    makeActionMessage(actionName, data = {}) {
        return {
            type: this.types.action,
            data: {
                ...data,
                action_name: actionName,
            },
        };
    },
    makeProcessMessage(target, action) {
        return {
            type: this.types.process,
            data: {
                target: target,
                action: action,
            },
        };
    },
    makeControllerMessage(group, action, data = {}) {
        return {
            type: this.types.controller,
            data: {
                ...data,
                group: group,
                action: action,
            },
        };
    },
});
