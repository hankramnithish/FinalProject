# import required python libraries
import rclpy
from rclpy.node import Node
import numpy as np
import math

# import required messages
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ReadingLaser(Node):
    def __init__(self):
        super().__init__('reading_laser')

        # create an object to subscribe /scan topic
        self.laser_subscriber= self.create_subscription(LaserScan,'/scan',self.laser_callback,10)
        
        # create objects for /fakescan and /laser_avoid ('/fakescan has laserscan data in a predefined range and /laser_avoid has the obstacle status')
        self.fakescan_publisher = self.create_publisher(LaserScan, '/fakescan', 10)
        self.obstacle_status_publisher = self.create_publisher(Bool, '/laser_avoid' , 10)
        self.obstacle_status = Bool()

        # create timers for calling fake_laser_callback and obstacle_status_callback functions
        self.fake_laser_callback_timer = self.create_timer(0.2, self.fake_laser_callback)
        self.obstacle_status_callback_timer = self.create_timer(0.2, self.obstacle_status_callback)
        
        # initialize the variables
        self.laser_data = None
        self.filtered_laser_data = None
        self.minimum_obstacle_detection_distance = 0.4
        self.warnings_array = 0
        self.total_no_of_warnings = 0
        
        # initialize the counter and flags
        self.obstacle_status_counter = 0
        self.laser_detect_flag = 0

    def laser_callback(self,laser_msg):
        # receives full laser data and filters specified range of laser data
        self.laser_data = laser_msg
        laser_point_data = np.array(self.laser_data.ranges)
        index = np.arange(90,270,dtype=int)
        self.filtered_laser_data = laser_point_data[index]
        self.laser_detect_flag = 1
        

    def fake_laser_callback(self):
        if self.laser_data != None:
            # Publishes specified range of laser data
            laser_scan = LaserScan()
            laser_scan.header.stamp = self.laser_data.header.stamp
            laser_scan.header.frame_id = self.laser_data.header.frame_id
            laser_scan.angle_min = self.laser_data.angle_min
            laser_scan.angle_max = self.laser_data.angle_max
            laser_scan.angle_increment = self.laser_data.angle_increment
            laser_scan.time_increment = self.laser_data.time_increment
            laser_scan.range_min = self.laser_data.range_min
            laser_scan.range_max = self.laser_data.range_max
            laser_scan.ranges = self.filtered_laser_data.tolist()
            laser_scan.intensities = self.laser_data.intensities
            self.fakescan_publisher.publish(laser_scan)
        #self.get_logger().info('Publishing'%laser_scan.ranges)

    def obstacle_status_callback(self):
        # this function publishes obstacle status
        if self.laser_detect_flag:
            self.warnings_array = self.filtered_laser_data <= self.minimum_obstacle_detection_distance # all the filtered_laser_data <= minimum_obstacle_detection_distance will be saved in warnings_array in the form of True value
            self.total_no_of_warnings = self.warnings_array.sum() # total number of Trues in warnings_array will be counted
            if self.total_no_of_warnings > 3:
                # if the total_no_of_warnings is atleast greater than 3, it is considered to be an obstacle
                self.obstacle_status_counter = self.obstacle_status_counter+1   # obstacle_status_counter is used to avoid false trigger
                if self.obstacle_status_counter >=1:
                    # the above threshold value (>=1) can be changed based on the robustness needed
                    self.obstacle_status.data = True
                    self.obstacle_status_publisher.publish(self.obstacle_status)
                else:
                    self.obstacle_status.data =False
                    self.obstacle_status_publisher.publish(self.obstacle_status)          
            else:
                self.obstacle_status_counter = 0
                self.obstacle_status.data = False
                self.obstacle_status_publisher.publish(self.obstacle_status)           


def main(args=None):
    rclpy.init()
    reading_laser = ReadingLaser()                  
    reading_laser.get_logger().info("Laser laser_scanner")
    rclpy.spin(reading_laser)
    reading_laser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
