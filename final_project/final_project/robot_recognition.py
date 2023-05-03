# import required python libraries
import torch
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np

# import required messages
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('yolov5_node')
        # create an object to subscribe /color/preview/image
        self.image_subscriber = self.create_subscription(Image,'color/preview/image',self.listener_callback,10)

        # create an object to publish the predicted image data
        self.image_prediction_data_publisher = self.create_publisher(Int32MultiArray, '/image_prediction', 10)
        self.image_prediction_data = Int32MultiArray()
        self.br = CvBridge()     # create an object for CvBridge(). CvBridge is used for converting image data from ros2 format to cv2 format
        
        self.model = torch.hub.load('ultralytics/yolov5','custom','/home/nithish/nithish_ws/src/final_project/final_project/best.pt') # load the yolov5 model and weights
        self.get_logger().info("Node Initialized")

        # set the variables for cv2 functions
        self.text_color = (0,255,255)
        self.box_color = (100,0,255)
        self.text_thickness = 1
        self.box_thickness = 2 
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.fontScale_for_classes = 0.5
        self.fontScale_for_confidence_score = 0.45

        # dictionary to map class numbers to class names
        self.class_names = {0:'bump',1:'leftTurn',2:'leftUTurn',3:'rightTurn',4:'rightUTurn',5:'speed_10',6:'speed_20',7:'speed_30',8:'stop',9:'straight'}

    def listener_callback(self, data):
        #self.get_logger().info("Got Image")
        current_frame = self.br.imgmsg_to_cv2(data)  # convert from ros2 format to cv2 format
        processed_image = self.model(current_frame)  # model prediction
        result = processed_image.pandas().xyxy       # convert the result to pandas dataframe
        if (not(result[0]['class'].empty)): 
            # this if condition is to avoid error when a road signal is not detected
            index_number = np.argmax(result[0]['xmax'].values-result[0]['xmin'].values)  # if there are multiple road signs in an image, there will more than one class. So the class with the largest bounding box needs to be found and assigned to this variable
            # unpack x_min, y_min, x_max, y_max from the pandas dataframe
            x_min = int(result[0]['xmin'][index_number])  
            y_min = int(result[0]['ymin'][index_number])
            x_max = int(result[0]['xmax'][index_number])
            y_max = int(result[0]['ymax'][index_number])
            class_number = int(result[0]['class'][index_number])
            confidence_score = result[0]['confidence'][index_number]
            start_point = (x_min,y_min)   
            end_point = (x_max,y_max)
            # draw bounding box of the prediction on the output image
            image = cv2.putText(current_frame, self.class_names[class_number], (x_min,y_min-5), self.font, self.fontScale_for_classes, self.text_color, self.text_thickness, cv2.LINE_AA)
            image = cv2.putText(current_frame, str(round(confidence_score,3)), (x_min,y_max+15), self.font, self.fontScale_for_confidence_score, self.text_color, self.text_thickness, cv2.LINE_AA)
            image = cv2.rectangle(current_frame, start_point, end_point, self.box_color, self.box_thickness)

            if confidence_score>=0.6:
                # publish the image_prediction_data only if the confidence_score is greater than 0.6            
                self.image_prediction_data.data = [class_number, x_min, x_max]
                self.image_prediction_data_publisher.publish(self.image_prediction_data)
        else:
            # if there is no roadsign just output the current frame
            image = current_frame
        cv2.imshow("Image", image)
        cv2.waitKey(10)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
