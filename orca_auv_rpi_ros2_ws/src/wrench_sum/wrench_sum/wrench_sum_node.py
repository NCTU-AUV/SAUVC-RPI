import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
import numpy as np

class WrenchSum(Node):
    def __init__(self):
        super().__init__('wrench_sum')

        # --- 1. Declare parameters ---
        # Default input topic list (modify in launch file)
        self.declare_parameter('input_topics', ['/thruster_front/wrench', '/thruster_back/wrench'])
        # Output topic name
        self.declare_parameter('output_topic', '/auv/net_wrench')
        # Publish rate (Hz)
        self.declare_parameter('publish_rate', 20.0)

        # Get parameter values
        self.input_topics = self.get_parameter('input_topics').get_parameter_value().string_array_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        pub_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # --- 2. Initialize storage ---
        # Use Dictionary to store the latest data for each topic
        # Key: topic_name, Value: np.array([fx, fy, fz, tx, ty, tz])
        self.wrench_buffer = {topic: np.zeros(6) for topic in self.input_topics}
        
        # --- 3. Create Subscribers ---
        self.subs = []
        for topic in self.input_topics:
            # Use lambda to capture the current topic name
            self.subs.append(
                self.create_subscription(
                    Wrench,
                    topic,
                    lambda msg, t=topic: self.listener_callback(msg, t),
                    10
                )
            )
            self.get_logger().info(f'Subscribed to: {topic}')

        # --- 4. Create Publisher ---
        self.publisher_ = self.create_publisher(Wrench, output_topic, 10)

        # --- 5. Create Timer ---
        self.timer = self.create_timer(1.0 / pub_rate, self.timer_callback)

    def listener_callback(self, msg, topic_name):
        """
        Update Buffer value when receiving Wrench from any source
        """
        # Apply NumPy format
        wrench_arr = np.array([
            msg.force.x,
            msg.force.y,
            msg.force.z,
            msg.torque.x,
            msg.torque.y,
            msg.torque.z
        ])
        
        # Update the latest value for this topic
        self.wrench_buffer[topic_name] = wrench_arr

    def timer_callback(self):
        """
        Periodically sum all Wrench in Buffer and publish
        """
        # 1. Calculate net force (Element-wise sum)
        # Sum all values
        net_wrench_arr = sum(self.wrench_buffer.values())

        # 2. Build output message
        msg = Wrench()
        msg.force.x = net_wrench_arr[0]
        msg.force.y = net_wrench_arr[1]
        msg.force.z = net_wrench_arr[2]
        msg.torque.x = net_wrench_arr[3]
        msg.torque.y = net_wrench_arr[4]
        msg.torque.z = net_wrench_arr[5]

        # 3. Publish
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WrenchSum()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
