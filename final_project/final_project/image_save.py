#!usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
class ImageSubscriber(Node):
  def __init__(self):
    super().__init__('subscriber')
    self.timer = self.create_timer(1.5, self.save_images)
    self.subscription = self.create_subscription(Image,'/color/preview/image', self.listener_callback, 10)
    self.br = CvBridge()
    self.i =83
  def listener_callback(self, data):
    self.get_logger().info('Receiving video frame')
    self.current_frame = self.br.imgmsg_to_cv2(data)
    #self.current_frame = cv2.resize(self.current_frame, (640, 360))
    #cv2.imshow("camera", self.current_frame)
    cv2.waitKey(1)
    
  def save_images(self):
    if self.i < 110:
      self.i = self.i+1
      #imestr = time.strftime("%Y%m%d-%H%M%S")
      current_frame_path = "/home/hankwang/turtlebot_ws/src/sign_testing/sign_testing/bump-2_" + str(self.i)+".jpg"
      print(f"Image {self.i} is saved ")
      cv2.imwrite(current_frame_path, self.current_frame)
    
    
def main(args=None):

  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
