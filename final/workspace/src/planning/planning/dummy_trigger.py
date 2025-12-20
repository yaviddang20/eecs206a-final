import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class DummyStepDoneService(Node):
    def __init__(self):
        super().__init__('dummy_step_done_service')

        self.declare_parameter('settle_time', 0.25)
        self.settle_time = self.get_parameter('settle_time').value

        self.srv = self.create_service(
            Trigger,
            '/external_step_done',
            self.handle_trigger
        )

        self.get_logger().info(
            f'Dummy service ready. Sleeping {self.settle_time}s per call'
        )

    def handle_trigger(self, request, response):
        self.get_logger().info('Dummy trigger received')

        time.sleep(self.settle_time)

        response.success = True
        response.message = 'Dummy step done'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DummyStepDoneService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
