#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class MyProcessing:
    
    def __init__(self):
        rospy.init_node('itu_rover_CompressedVideo_listener', anonymous=False)
        self.filter_subscriber = rospy.Subscriber('/filter', String, self.FilteredCommandCallback)
        self.compressedVideo_subscriber = rospy.Subscriber('/video_topic/compressed', CompressedImage, self.CompressedImageCallback)
        self.compressedVideo_publisher = rospy.Publisher('/rover_view/compressed', CompressedImage, queue_size=10)
        self.rate = rospy.Rate(0.5) 
        self.bridge = CvBridge()
        self.current_image = None
    

    def CompressedImageCallback(self, image_msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        # Check if the image is None before setting it
        if cv_image is not None:
            # Store the current image for processing
            self.current_image = cv_image
            # After processing, convert the image back to CompressedImage
        else:
            rospy.logwarn("Received None image. Skipping processing.")
        

    def FilteredCommandCallback(self, data):
        command = data.data
        image = self.current_image
        if command == 'GRAY':
            self.process_image(self.grayscale, image)
        elif command == 'RGB':
            self.process_image(self.rgb, image)
        elif command == 'RESIZE_UP':
            self.process_image(self.resize_up, image)
        elif command == 'RESIZE_DOWN':
            self.process_image(self.resize_down, image)
        else:
            rospy.logwarn(f"Unknown command received: {command}")
    
 
    def process_image(self, operation, image):
        # Apply the specified operation
        processed_image = operation(image)

        # Publish the processed image
        self.publish_processed_image(processed_image)

   
    def grayscale(self, image):
        if image is None:
            rospy.logwarn("Image is None. Skipping grayscale.")
            return image
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    def rgb(self, image):
        # Do nothing, as it's already in RGB format
        return image

    def resize_up(self, image):
        if image is None:
            rospy.logwarn("Image is None. Skipping resize_up.")
            return image

        print(type(image))
        
        height, width = self.current_image.shape[:2]
        print(height,width)

        new_height = height * 2
        new_width = width * 2
        

        rospy.loginfo(f"Resizing up: Original size: ({height}, {width}), New size: ({new_height}, {new_width})")
        

        resized_image = cv2.resize(self.current_image, (new_width, new_height), interpolation=cv2.INTER_NEAREST)
        return resized_image


    def resize_down(self, image):
        if image is None:
            rospy.logwarn("Image is None. Skipping resize_down.")
            return image
        

        height, width = self.current_image.shape[:2]
        print(height,width)

        new_height = height // 2
        new_width = width // 2
        

        rospy.loginfo(f"Resizing down: Original size: ({height}, {width}), New size: ({new_height}, {new_width})")
        

        resized_image = cv2.resize(self.current_image, (new_width, new_height), interpolation=cv2.INTER_NEAREST)
        return resized_image
        

    def publish_processed_image(self, image):
        # Publish the processed image as a CompressedImage message
        image_msg = self.bridge.cv2_to_compressed_imgmsg(image)
        self.compressedVideo_publisher.publish(image_msg)



if __name__ == '__main__':
    print("my_processing.py node has been started...")
    FilteredCommandNode = MyProcessing()
    rospy.spin()  # Keep the script running





