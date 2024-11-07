#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class potatoDetectionNode:

    def __init__(self):
        rospy.init_node("potato_detection_node")
        self.counter_ = 0
        self.model = YOLO('/home/divyanthlg/workspaces/potato_ws/src/motor_control/potato_detection_yolov8.pt')
        self.color_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.callback_color_image)
        self.depth_subscriber = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.callback_depth_image)
        rospy.loginfo("Subscribed to color and depth image topic")
        self.bridge = CvBridge()
        self.potato_color_publisher = rospy.Publisher('potato_detection_image', Image, queue_size=10)
        self.potato_depth_publisher = rospy.Publisher('potato_location', Point, queue_size=10)
        self.results = None

    def callback_color_image(self, color_message):
        # Process the image data here
        color_image = self.bridge.imgmsg_to_cv2(color_message, desired_encoding='bgr8')
        self.results = self.model.predict(color_image,device='0',conf=0.25)

        im_array = self.results[0].plot()
        detection_image = self.bridge.cv2_to_imgmsg(im_array)
        self.potato_color_publisher.publish(detection_image)

        # Log some information
        rospy.loginfo("Received color image message. Width: {}, Height: {}".format(color_message.width, color_message.height))
            
    def callback_depth_image(self, depth_message):
        
        depth_data = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding='passthrough')
        depth_array = np.array(depth_data, dtype=np.float32)

        if self.results is not None:
            for result in self.results:        
                boxes = self.results[0].boxes.xyxy.tolist()
                classes = self.results[0].boxes.cls.tolist()
                names = self.results[0].names
                confidences = self.results[0].boxes.conf.tolist()
        
                for box, cls, conf in zip(boxes, classes, confidences):
                    x1, y1, x2, y2 = box
                    confidence = conf
                    detected_class = cls
                    name = names[int(cls)]
                    object_x = int((y1+y2)/2)
                    object_y = int((x1+x2)/2)
                    object_depth = depth_array[object_x][object_y]
                    
                    point_msg = Point()
                    point_msg.x = object_x
                    point_msg.y = object_y
                    point_msg.z = object_depth
                    
                    self.potato_depth_publisher.publish(point_msg)
                    rospy.loginfo("Published Point at (x: {}, y: {}, depth: {})".format(object_x, object_y, object_depth))
	
                self.results = None

def main(args=None):
    potato_detection_node = potatoDetectionNode()
    rospy.spin()
    
if __name__ == "__main__":
    main()
