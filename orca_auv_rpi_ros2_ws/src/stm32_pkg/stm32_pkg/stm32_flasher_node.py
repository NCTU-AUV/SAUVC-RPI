import os
import subprocess
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class Stm32FlasherNode(Node):
    def __init__(self) -> None:
        super().__init__('stm32_flasher_node')

        self.declare_parameter('bin_path', '/root/orca_auv_rpi_ros2_ws/stm32_binary/SAUVC2024.bin')
        self.declare_parameter('flash_address', '0x08000000')
        self.declare_parameter('st_flash_cmd', 'st-flash')

        self._service = self.create_service(Trigger, 'flash_stm32', self._handle_flash)
        self.get_logger().info('STM32 flasher node ready. Service: /flash_stm32')

    def _handle_flash(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        _ = request
        bin_path = self.get_parameter('bin_path').get_parameter_value().string_value
        flash_address = self.get_parameter('flash_address').get_parameter_value().string_value
        st_flash_cmd = self.get_parameter('st_flash_cmd').get_parameter_value().string_value

        resolved_bin = self._resolve_bin_path(bin_path)
        if resolved_bin is None:
            response.success = False
            response.message = f'Binary not found: {bin_path}'
            self.get_logger().error(response.message)
            return response

        cmd = [st_flash_cmd, '--reset', 'write', resolved_bin, flash_address]
        self.get_logger().info(f'Running: {" ".join(cmd)}')

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=False)
        except FileNotFoundError:
            response.success = False
            response.message = f'Command not found: {st_flash_cmd}'
            self.get_logger().error(response.message)
            return response
        except Exception as exc:
            response.success = False
            response.message = f'Failed to run flash command: {exc}'
            self.get_logger().error(response.message)
            return response

        if result.returncode == 0:
            response.success = True
            response.message = (result.stdout or 'Flash completed successfully.').strip()
            self.get_logger().info(response.message)
            return response

        stderr = (result.stderr or '').strip()
        stdout = (result.stdout or '').strip()
        response.success = False
        response.message = f'Flash failed (code {result.returncode}). {stderr or stdout}'.strip()
        self.get_logger().error(response.message)
        return response

    def _resolve_bin_path(self, bin_path: str) -> Optional[str]:
        path = Path(bin_path)
        if path.is_file():
            return str(path)

        repo_root = self._find_repo_root()
        if repo_root is not None:
            candidate = repo_root / bin_path
            if candidate.is_file():
                return str(candidate)

        return None

    def _find_repo_root(self) -> Optional[Path]:
        current = Path(__file__).resolve()
        for parent in [current] + list(current.parents):
            if (parent / 'SAUVC-STM32').is_dir():
                return parent
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Stm32FlasherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
