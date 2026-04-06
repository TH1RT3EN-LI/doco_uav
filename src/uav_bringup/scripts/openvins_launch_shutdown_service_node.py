#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class OpenvinsLaunchShutdownServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("openvins_launch_shutdown_service_node")

        self.declare_parameter("shutdown_service", "/uav/openvins/command/shutdown")
        self.declare_parameter("shutdown_delay_s", 0.05)

        self._shutdown_service = str(self.get_parameter("shutdown_service").value)
        self._shutdown_delay_s = max(
            0.0, float(self.get_parameter("shutdown_delay_s").value)
        )

        self._shutdown_requested = False
        self._shutdown_timer = None

        self._service = self.create_service(
            Trigger, self._shutdown_service, self._handle_shutdown
        )

        self.get_logger().info(
            f"openvins launch shutdown service ready: {self._shutdown_service}"
        )

    def _handle_shutdown(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request

        if self._shutdown_requested:
            response.success = True
            response.message = "openvins launch shutdown already requested"
            return response

        self._shutdown_requested = True
        delay_s = max(0.001, self._shutdown_delay_s)
        self._shutdown_timer = self.create_timer(delay_s, self._perform_shutdown)

        self.get_logger().warn(
            f"received shutdown request on {self._shutdown_service}; "
            f"launch shutdown will start in {delay_s:.3f}s"
        )
        response.success = True
        response.message = "openvins launch shutdown requested"
        return response

    def _perform_shutdown(self) -> None:
        if self._shutdown_timer is not None:
            self._shutdown_timer.cancel()
            self._shutdown_timer = None

        self.get_logger().error("shutting down openvins launch by request")
        rclpy.shutdown()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = OpenvinsLaunchShutdownServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
