from rclpy.node import Node
from std_srvs.srv import Trigger


class Stm32AutoFlasher:
    def __init__(self, node: Node, set_status):
        self._node = node
        self._set_status = set_status
        self._client = node.create_client(
            Trigger,
            node.get_parameter("stm32_flash_service").value,
        )
        self._started = False
        self._finished = False
        self._start_stamp = node.get_clock().now()
        self._timer = node.create_timer(0.5, self._maybe_flash)

    def _maybe_flash(self):
        if not self._node.get_parameter("auto_flash_stm32_on_startup").value:
            self._finished = True
            self._timer.cancel()
            return

        if self._started or self._finished:
            return

        if self._client.service_is_ready():
            self._started = True
            self._set_status("STM32 auto-flash started")
            self._node.get_logger().info("STM32 auto-flash started")
            future = self._client.call_async(Trigger.Request())
            future.add_done_callback(self._on_flash_result)
            return

        timeout_s = float(
            self._node.get_parameter("stm32_flash_service_timeout_s").value
        )
        if self._age_s(self._start_stamp) > timeout_s:
            self._finished = True
            self._timer.cancel()
            service_name = self._node.get_parameter("stm32_flash_service").value
            status = f"STM32 auto-flash skipped: {service_name} service not ready"
            self._set_status(status)
            self._node.get_logger().warning(status)

    def _on_flash_result(self, future):
        self._finished = True
        self._timer.cancel()

        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            status = f"STM32 auto-flash failed: {exc}"
            self._set_status(status)
            self._node.get_logger().warning(status)
            return

        if response.success:
            status = "STM32 auto-flash succeeded"
            if response.message:
                status = f"{status}: {response.message}"
            self._set_status(status)
            self._node.get_logger().info(status)
            return

        status = "STM32 auto-flash failed"
        if response.message:
            status = f"{status}: {response.message}"
        self._set_status(status)
        self._node.get_logger().warning(status)

    def _age_s(self, stamp):
        return (self._node.get_clock().now() - stamp).nanoseconds / 1e9
