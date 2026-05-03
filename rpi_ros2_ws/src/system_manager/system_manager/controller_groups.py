from lifecycle_msgs.msg import State
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from std_srvs.srv import Trigger


class AsyncLifecycleClient:
    """Minimal async lifecycle client for ROS 2 Humble."""

    def __init__(self, node: Node, node_name: str):
        self._change_state_client = node.create_client(
            ChangeState,
            f"{node_name}/change_state",
        )
        self._get_state_client = node.create_client(
            GetState,
            f"{node_name}/get_state",
        )

    def service_is_ready(self) -> bool:
        return (
            self._change_state_client.service_is_ready()
            and self._get_state_client.service_is_ready()
        )

    def get_state(self):
        return self._get_state_client.call_async(GetState.Request())

    def change_state(self, transition_id: int):
        request = ChangeState.Request()
        request.transition.id = transition_id
        return self._change_state_client.call_async(request)


class ControllerGroupManager:
    def __init__(self, node: Node, controller_groups):
        self._node = node
        self._controller_groups = controller_groups
        self._lifecycle_clients = {}
        self._reset_clients = {}

    def enable_group(self, group: str):
        for node_name in self._controller_groups.get(group, []):
            self._request_lifecycle_active(node_name)

    def disable_group(self, group: str):
        for node_name in self._controller_groups.get(group, []):
            self._request_lifecycle_inactive(node_name)

    def disable_all(self):
        for group in self._controller_groups:
            self.disable_group(group)

    def reset_group(self, group: str):
        for node_name in self._controller_groups.get(group, []):
            client = self._get_reset_client(node_name)
            if not client.service_is_ready():
                self._node.get_logger().warning(f"Reset service not ready for {node_name}")
                continue
            future = client.call_async(Trigger.Request())
            future.add_done_callback(lambda f, n=node_name: self._log_reset_result(n, f))

    def reset_all(self):
        for group in self._controller_groups:
            self.reset_group(group)

    def _get_lifecycle_client(self, node_name: str):
        client = self._lifecycle_clients.get(node_name)
        if client is None:
            client = AsyncLifecycleClient(self._node, node_name)
            self._lifecycle_clients[node_name] = client
        return client

    def _request_lifecycle_active(self, node_name: str):
        client = self._get_lifecycle_client(node_name)
        if not client.service_is_ready():
            self._node.get_logger().warning(f"Lifecycle services not ready for {node_name}")
            return

        future = client.get_state()
        future.add_done_callback(lambda f, n=node_name: self._activate_from_state(n, f))

    def _activate_from_state(self, node_name: str, future):
        try:
            state_id = future.result().current_state.id
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(
                f"Failed to get lifecycle state for {node_name}: {exc}"
            )
            return

        if state_id == State.PRIMARY_STATE_ACTIVE:
            return
        if state_id == State.PRIMARY_STATE_INACTIVE:
            self._change_lifecycle_state(node_name, Transition.TRANSITION_ACTIVATE)
            return
        if state_id == State.PRIMARY_STATE_UNCONFIGURED:
            future = self._change_lifecycle_state(
                node_name,
                Transition.TRANSITION_CONFIGURE,
                log_result=False,
            )
            if future is not None:
                future.add_done_callback(
                    lambda f, n=node_name: self._activate_after_configure(n, f)
                )
            return

        self._node.get_logger().warning(
            f"Cannot activate {node_name} from lifecycle state id {state_id}"
        )

    def _activate_after_configure(self, node_name: str, future):
        if not self._lifecycle_transition_succeeded(
            node_name,
            "configure",
            future,
        ):
            return
        self._change_lifecycle_state(node_name, Transition.TRANSITION_ACTIVATE)

    def _request_lifecycle_inactive(self, node_name: str):
        client = self._get_lifecycle_client(node_name)
        if not client.service_is_ready():
            self._node.get_logger().warning(f"Lifecycle services not ready for {node_name}")
            return

        future = client.get_state()
        future.add_done_callback(lambda f, n=node_name: self._deactivate_from_state(n, f))

    def _deactivate_from_state(self, node_name: str, future):
        try:
            state_id = future.result().current_state.id
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(
                f"Failed to get lifecycle state for {node_name}: {exc}"
            )
            return

        if state_id == State.PRIMARY_STATE_ACTIVE:
            self._change_lifecycle_state(node_name, Transition.TRANSITION_DEACTIVATE)
            return
        if state_id in (
            State.PRIMARY_STATE_UNCONFIGURED,
            State.PRIMARY_STATE_INACTIVE,
            State.PRIMARY_STATE_FINALIZED,
        ):
            return

        self._node.get_logger().warning(
            f"Cannot deactivate {node_name} from lifecycle state id {state_id}"
        )

    def _change_lifecycle_state(
        self,
        node_name: str,
        transition_id: int,
        log_result: bool = True,
    ):
        client = self._get_lifecycle_client(node_name)
        if not client.service_is_ready():
            self._node.get_logger().warning(f"Lifecycle services not ready for {node_name}")
            return None

        future = client.change_state(transition_id)
        if log_result:
            future.add_done_callback(
                lambda f, n=node_name, t=transition_id: self._log_lifecycle_result(n, t, f)
            )
        return future

    def _get_reset_client(self, node_name: str):
        client = self._reset_clients.get(node_name)
        if client is None:
            client = self._node.create_client(Trigger, f"{node_name}/reset")
            self._reset_clients[node_name] = client
        return client

    def _log_lifecycle_result(self, node_name: str, transition_id: int, future):
        self._lifecycle_transition_succeeded(node_name, str(transition_id), future)

    def _lifecycle_transition_succeeded(self, node_name: str, label: str, future) -> bool:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(
                f"Lifecycle transition {label} failed on {node_name}: {exc}"
            )
            return False
        if not response.success:
            self._node.get_logger().warning(
                f"Lifecycle transition {label} rejected by {node_name}"
            )
            return False
        return True

    def _log_reset_result(self, node_name: str, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(f"Reset call failed on {node_name}: {exc}")
            return
        if not response.success:
            self._node.get_logger().warning(
                f"Reset failed on {node_name}: {response.message}"
            )
