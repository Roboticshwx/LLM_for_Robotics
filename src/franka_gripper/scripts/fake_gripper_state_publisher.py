import rclpy
from rclpy import Parameter
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FakeGripperStatePublisher(Node):

    def __init__(self):
        super().__init__('fake_gripper_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '~/joint_states', 1)
        timer_period = 0.1  # seconds

        self.declare_parameter('joint_names', Parameter.Type.STRING_ARRAY)
        self.joint_names = (
            self.get_parameter('joint_names').get_parameter_value().string_array_value
        )

        assert len(self.joint_names) == 2
        self.timer = self.create_timer(timer_period, self.publish_state)

    def publish_state(self):
        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = self.joint_names
        joint_states.position = [0.035, 0.035]
        joint_states.velocity = [0.0, 0.0]
        joint_states.effort = [0.0, 0.0]
        self.publisher_.publish(joint_states)


def main(args=None):
    rclpy.init(args=args)

    state_publisher = FakeGripperStatePublisher()

    rclpy.spin(state_publisher)
    state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()