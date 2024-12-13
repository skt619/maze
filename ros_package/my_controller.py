import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

# Constants
LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX = 150
LEFT_SIDE_INDEX = 90

class RandomWalk(Node):
    def __init__(self):
        super().__init__('random_walk_node')
        
        # Node variables
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.laser_forward = 0
        self.pose_saved = None
        self.cmd = Twist()

        # ROS2 publishers and subscribers
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan, '/scan', self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.subscriber2 = self.create_subscription(
            Odometry, '/odom', self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Timer callback for control loop
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback1(self, msg1):
        scan = msg1.ranges
        self.scan_cleaned = [min(3.5, max(0.0, r if r != float('Inf') and not math.isnan(r) else 3.5)) for r in scan]

    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        self.pose_saved = position

    def timer_callback(self):
        if not self.scan_cleaned:
            self.turtlebot_moving = False
            return

        # Get minimum LIDAR readings
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        if front_lidar_min < SAFE_STOP_DISTANCE:
            # Stop and decide next action
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = False
            self.get_logger().info('Robot stopped')

            if right_lidar_min > left_lidar_min:
                self.cmd.angular.z = -0.5
                self.get_logger().info('Turning right')
            else:
                self.cmd.angular.z = 0.5
                self.get_logger().info('Turning left')
            self.publisher_.publish(self.cmd)

        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
            # Slow down and adjust
            self.cmd.linear.x = 0.07
            self.cmd.angular.z = -0.5 if right_lidar_min > left_lidar_min else 0.5
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Navigating around obstacle')

        elif self.stall:
            # Reverse and turn when stalled
            self.cmd.linear.x = -0.1
            self.cmd.angular.z = 0.5
            self.publisher_.publish(self.cmd)
            self.get_logger().warning('Stall detected: Reversing')

        else:
            # Move forward
            self.cmd.linear.x = LINEAR_VEL
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            self.get_logger().info('Moving forward')

        self.get_logger().info(f'Front Obstacle Distance: {front_lidar_min:.2f}')

def main(args=None):
    rclpy.init(args=args)
    try:
        random_walk_node = RandomWalk()
        rclpy.spin(random_walk_node)
    except Exception as e:
        random_walk_node.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        random_walk_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
