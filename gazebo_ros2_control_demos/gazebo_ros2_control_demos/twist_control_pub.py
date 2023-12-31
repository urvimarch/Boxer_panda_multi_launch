#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener
from tf2_ros import Buffer
import time
import math
import matplotlib.pyplot as plt

class CmdVelController(Node):
    def __init__(self):
        super().__init__('cmd_vel_controller')
        self.publisher = self.create_publisher(Twist, '/diff_drive_base_controller/cmd_vel_unstamped', 10)
        #self.odometry_subscriber = self.create_subscription(Odometry, '/diff_drive_base_controller/odom', self.odometry_callback, 10)
   
        self.timer = self.create_timer(0.05, self.publish_twist)  # Publish every 1 second
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.start_time = time.time()
        self.duration = 70  # 1 minute
        self.initial_twist = Twist()
        self.initial_twist.linear.x = 0.3
        self.initial_twist.angular.z = 0.1
        self.x_r = 0.0
        self.y_r = 0.0
        self.theta_r = 0.0
        self.e1_values = []
        self.e2_values = []
        self.e3_values = []
        self.timestamps = []


    def publish_twist(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        #print(self.start_time,current_time,elapsed_time)

        if elapsed_time < self.duration:
            if self.theta_r > math.pi:
                self.theta_r -= 2 * math.pi
            elif self.theta_r < -math.pi:
                self.theta_r += 2 * math.pi
            self.theta_r += self.initial_twist.angular.z* 0.05               
            self.x_r += 0.3*math.cos(self.theta_r) * 0.05
            self.y_r += 0.3*math.sin(self.theta_r) * 0.05  
           
        
            # You can also obtain and compare odometry and calculate the error here

            odometry_msg = self.tf_buffer.lookup_transform('odom', 'chassis_link', rclpy.time.Time())

            # Extract translation values (x, y, z)
            x = odometry_msg.transform.translation.x
            y = odometry_msg.transform.translation.y
            z = odometry_msg.transform.translation.z

            quaternion = (
                odometry_msg.transform.rotation.x,
                odometry_msg.transform.rotation.y,
                odometry_msg.transform.rotation.z,
                odometry_msg.transform.rotation.w
             )

            yaw = math.atan2(2.0 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1]),
                         1.0 - 2.0 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]))
            
            #print(self.theta_r, self.x_r, self.y_r, yaw,x, y,)

            # Calculate error and adjust twist_msg based on odometry
            e1 = math.cos(yaw)*(self.x_r-x) + math.sin(yaw)*(self.y_r-y)
            e2 = -math.sin(yaw)*(self.x_r-x) + math.cos(yaw)*(self.y_r-y)
            e3 = self.theta_r - yaw

            print(e1,e2,e3)
            # Collect values and timestamps
            self.e1_values.append(e1)
            self.e2_values.append(e2)
            self.e3_values.append(e3)
            self.timestamps.append(current_time - self.start_time)

            k1 = 0.120 ; k2= 0.150 ; k3= 0.190

            # Create the Twist message based on initial values
            twist_msg = Twist()
            twist_msg.linear.x = self.initial_twist.linear.x*math.cos(e3) + (k1*e1)
            twist_msg.angular.z = self.initial_twist.angular.z +  self.initial_twist.linear.x*k2*e2 + k3*math.sin(e3)
            
            print(twist_msg.linear.x,twist_msg.angular.z)
            # Publish the Twist message
            self.publisher.publish(twist_msg)
            self.get_logger().info('Publishing Twist message')
        else:
            self.get_logger().info('Controller finished after {} seconds.'.format(elapsed_time))
            self.timer.cancel()
            # Plot e1, e2, and e3 against time
            plt.figure()   #Optional: Set the figure size

            plt.plot(self.timestamps, self.e1_values, label='e1')
            plt.plot(self.timestamps, self.e2_values, label='e2')
            plt.plot(self.timestamps, self.e3_values, label='e3')

            # Add labels and a legend
            plt.xlabel('Time (s)')
            plt.ylabel('Error Values')
            plt.title('Error Values Over Time')
            plt.legend()

            # Show the plot
            plt.grid(True)  # Optional: Add a grid
            plt.show()
            #print(self.e1_values, self.e2_values,self.e3_values,self.timestamps)
                


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_controller = CmdVelController()
    rclpy.spin(cmd_vel_controller)
    cmd_vel_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
