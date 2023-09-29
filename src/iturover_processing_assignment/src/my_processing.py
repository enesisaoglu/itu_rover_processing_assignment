#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# A Global Command that is set by default 'RGB'...
Command = 'RGB'

class MyProcessing:
    
    def __init__(self):
        # init node...
        rospy.init_node('itu_rover_compressedVideo_listener', anonymous=False)
        # A subscriber to the filtered commands...
        self.filter_subscriber = rospy.Subscriber('/filter', String, self.FilteredCommandCallback)
        # A subscriber to the compressed video...
        self.compressedVideo_subscriber = rospy.Subscriber('/video_topic/compressed', CompressedImage, self.CompressedImageCallback)
        # A publisher to publish the each processed frame... 
        self.compressedVideo_publisher = rospy.Publisher('/rover_view/compressed', CompressedImage, queue_size=10)
        self.rate = rospy.Rate(0.5) # HZ...
        # Provides an interface between ROS and OpenCv...
        self.bridge = CvBridge() 
        # Set current image to None...
        self.current_image = None
        # Set current image size 
        self.current_image_height = 480
        self.current_image_width = 640
    
    # a function to handle messages that are being sent on '/video_topic/compressed' topic...
    def CompressedImageCallback(self, image_msg):
        try:
            # try to convert compressed image message that is being sent to the cv2
            cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        # Check if the image is None before setting it
        if cv_image is not None:
            # Store the current image for processing
            self.current_image = cv_image
        else:
            rospy.logwarn("Received None image. Skipping processing.")
        

    # a function to handle messages that are being sent on '/filter' topic...
    def FilteredCommandCallback(self, data):
        global Command
        # Set global command to the data that is being sent as random...
        Command = data.data

    # a function to handle process...
    def Run(self):
        while not rospy.is_shutdown():
            image = self.current_image
            print("Video is being processed as", Command)
            processed_image = None
            if image is not None:
                if Command == 'GRAY':
                    processed_image = self.grayscale(image)
                elif Command == 'RGB':
                    processed_image = self.rgb(image)
                elif Command == 'RESIZE_UP':
                    processed_image = self.resize_up(image)
                elif Command == 'RESIZE_DOWN':
                    processed_image = self.resize_down(image)
                else:
                    rospy.logwarn(f"Unknown command received: {Command}")

                if processed_image is not None:
                    # After processing, convert the image back to CompressedImage
                    self.publish_processed_image(processed_image)
    
    # A function to process the image to grayscale...
    def grayscale(self, image):
        if image is None:
            rospy.logwarn("Image is None. Skipping grayscale.")
            return image
        # Safe current sizes...
        image = cv2.resize(image, (self.current_image_height, self.current_image_width), interpolation=cv2.INTER_NEAREST)
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # A function to process the image to RGB...
    def rgb(self, image):
        # Safe current sizes...
        image = cv2.resize(image, (self.current_image_height, self.current_image_width), interpolation=cv2.INTER_NEAREST)
        # Do nothing, as it's already in RGB format...
        return image
    
    # A function to process height and width of the image to up...
    def resize_up(self, image):
        if image is None:
            rospy.logwarn("Image is None. Skipping resize_up.")
            return image
        
        self.current_image_height, self.current_image_width = image.shape[:2]
        rospy.loginfo(f"Resizing up: Original size: ({self.current_image_height}, {self.current_image_width}) ")

        self.current_image_height = self.current_image_height * 2
        self.current_image_width = self.current_image_width * 2


        rospy.loginfo(f"New size: ({self.current_image_height}, {self.current_image_width})")
        

        resized_image = cv2.resize(image, (self.current_image_height, self.current_image_width), interpolation=cv2.INTER_NEAREST)
        return resized_image

    # A function to process height and width of the image to down...
    def resize_down(self, image):
        if image is None:
            rospy.logwarn("Image is None. Skipping resize_down.")
            return image
        
        self.current_image_height, self.current_image_width = image.shape[:2]
        rospy.loginfo(f"Resizing up: Original size: ({self.current_image_height}, {self.current_image_width}) ")

        self.current_image_height = self.current_image_height // 2
        self.current_image_width = self.current_image_width // 2


        rospy.loginfo(f"New size: ({self.current_image_height}, {self.current_image_width})")
        

        resized_image = cv2.resize(image, (self.current_image_height, self.current_image_width), interpolation=cv2.INTER_NEAREST)
        return resized_image
        
    # A function to publish the processed image as a CompressedImage message...
    def publish_processed_image(self, image):
        image_msg = self.bridge.cv2_to_compressed_imgmsg(image)
        self.compressedVideo_publisher.publish(image_msg)

if __name__ == '__main__':
    print("itu_rover_compressedVideo_listener node has been started...")
    FilteredCommandNode = MyProcessing()
    FilteredCommandNode.Run()
    rospy.spin()  # Keep the script running
