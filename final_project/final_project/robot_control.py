#import required python libraries
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient

# import required messages
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

# import RotateAngle from irobot_create_msgs.action
from irobot_create_msgs.action import RotateAngle


class RobotController(Node):
	def __init__(self):
		super().__init__("ControllerNode")
		# create subscriber object for /image_prediction and /laser_obstacle_status topics
		self.image_prediction_subscriber = self.create_subscription(Int32MultiArray,'/image_prediction',self.process_callback,10)
		self.laser_subscriber = self.create_subscription(Bool,'/laser_avoid',self.collision_callback,10)
		
		# create publisher object for /cmd_vel topic
		self.velocity_publisher = self.create_publisher(Twist,"/cmd_vel",10)
		self.velocity_msg = Twist()                                          # create an object for Twist() message

		# create an ActionClient object for rotate_angle
		self._action_client = ActionClient(self, RotateAngle, 'rotate_angle')

		# create a timer object to call the control_robot function
		self.timer = self.create_timer(0.2, self.control_robot)

		# initialize the flag variables
		self.obstacle_status = False
		self.robot_control_flag = False
		self.robot_rotate_flag = True

		self.velocity_msg.linear.x = 0.2     # initialize velocity in m/s
		self.min_width = 45                  # predefined width in pixel. Will be used for comparison with the bounding box width

	def send_goal(self, angle, max_rotation_speed):
		# function to rotate the robot. The rotate functionality is provided by irobot_create3
	    goal_msg = RotateAngle.Goal()
	    goal_msg.angle = angle 
	    goal_msg.max_rotation_speed = max_rotation_speed
	    self._action_client.wait_for_server()
	    self._send_goal_future = self._action_client.send_goal_async(goal_msg)
	    self._send_goal_future.add_done_callback(self.goal_response_callback)

	def goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().info('Goal rejected :(')
		self.get_logger().info('Goal accepted :)')
		self._get_result_future = goal_handle.get_result_async()
		self._get_result_future.add_done_callback(self.get_result_callback)

	def get_result_callback(self, future):
		result = future.result().result
		self.get_logger().info('Result: {0}'.format(result))

	def process_callback(self,img_prediction):
		# extract data from image_prediction message and find the width
		self.output_class = img_prediction.data[0]  # Output class number
		self.x_min = img_prediction.data[1]         # x_min of the predicted bounding box
		self.x_max = img_prediction.data[2]         # x_max of the predicted bounding box
		self.width = self.x_max-self.x_min          # width of the predicted bounding box
		self.robot_control_flag = True              # robot_control_flag is used so that condition is checked only if new message is received. If robot_control_flag is not used, then even if new message is not received, the previous class number will be used and robot keeps moving 

	def collision_callback(self,obstacle_data):
		# update obstacle_status. If there is an obstacle then obstacle_status becomes True
		self.obstacle_status = obstacle_data.data
	
	def control_robot(self):
		# Robot controller
		if self.obstacle_status:
			# if the obstacle_status is True, set the linear x velocity to zero
			self.velocity_msg.linear.x = 0.0
			self.velocity_publisher.publish(self.velocity_msg)
		else:
		    # if the obstacle_status is False, then class number is checked and velocity command will be sent accordingly	
			if self.robot_control_flag:
				if self.output_class == 9:                            # straight
					self.robot_rotate_flag = True                     # set the robot_rotate_flag to True in all the if and elif conditions, only then the next rotate will work
					self.velocity_msg.linear.x = 0.2
					if self.width > self.min_width:                   # if width of predicted bounding box is greater than predefined min_width, stop the robot
						self.velocity_msg.linear.x = 0.0

				elif self.output_class == 8:                          # stop
					self.robot_rotate_flag = True
					self.velocity_msg.linear.x = 0.2
					if self.width > self.min_width-5:
						self.velocity_msg.linear.x = 0.0

				elif self.output_class == 7:                          # speed_30
					self.robot_rotate_flag = True
					self.velocity_msg.linear.x = 0.35
					if self.width > self.min_width:
						self.velocity_msg.linear.x = 0.0

				elif self.output_class == 6:                          # speed_20
					self.robot_rotate_flag = True
					self.velocity_msg.linear.x = 0.2
					if self.width > self.min_width:
						self.velocity_msg.linear.x = 0.0

				elif self.output_class == 5:                          # speed_10
					self.robot_rotate_flag = True
					self.velocity_msg.linear.x = 0.1
					if self.width > self.min_width:
						self.velocity_msg.linear.x = 0.0

				elif self.output_class == 4:                          # rightUTurn     
					if self.width<self.min_width:
						self.robot_rotate_flag = True
						self.velocity_msg.linear.x = 0.2
					elif(self.robot_rotate_flag == True):
						self.robot_rotate_flag = False                # set the robot_rotate_flag to False, so that send_goal is called just once
						self.send_goal(math.radians(-180.0),0.5)

				elif self.output_class == 3:                          # rightTurn
					if self.width<self.min_width:
						self.robot_rotate_flag = True
						self.velocity_msg.linear.x = 0.2
					elif(self.robot_rotate_flag == True):
						self.robot_rotate_flag = False
						self.send_goal(math.radians(-90.0),0.5)

				elif self.output_class == 2:                          # leftUTurn
					if self.width<self.min_width:
						self.robot_rotate_flag = True
						self.velocity_msg.linear.x = 0.2
					elif(self.robot_rotate_flag == True):
						self.robot_rotate_flag = False
						self.send_goal(math.radians(180.0),0.5)

				elif self.output_class == 1:                          # leftTurn
					if self.width<self.min_width:
						self.robot_rotate_flag = True
						self.velocity_msg.linear.x = 0.2
					elif(self.robot_rotate_flag == True):
						self.robot_rotate_flag = False
						self.send_goal(math.radians(90.0),0.5)

				elif self.output_class == 0:                          # bump
					self.robot_rotate_flag = True
					self.velocity_msg.linear.x = 0.05
					if self.width > self.min_width+30:
						# +30 is added on right side on relational operator to stop the robot little far from the stop sign. This is to avoid the false detection from the YOLO model when the camera is too close stop sign
						self.velocity_msg.linear.x = 0.0
				
				self.velocity_publisher.publish(self.velocity_msg)    # publish velocity message

		self.robot_control_flag = False                             

	

def main(args=None):
	rclpy.init(args=args)
	robot = RobotController()    # create an object for RobotController class
	rclpy.spin(robot)
	robot.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()

