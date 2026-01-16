#!/usr/bin/env python
# coding: utf-8
import rospy
import numpy as np
from supra_msgs.msg import FloatImage
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from ImageProcessor import ImageProcessor


class CommunicationManager:

    def __init__(self):
        self.path_data: str = ""                          # Folder + Datasetname e.g /folder/dataset001

        # ROS Variables
        self.current_image = []                     # Stores current image in np.array: See callback_get_image
        self.x_table = 0                            # image coordinate in table coordinates
        self.y_table = 0                            # image coordinate in table coordinates
        self.z_table = 0                            # image coordinate in table coordinates
        self.stored_images = []                     # List of current_images which be saved after appending
        self.stored_table_positions = []            # List of table_position list [x,y,z] after appending
        self.new_position_received = False
        self.already_position_requested = False     # Keep track if I already requested a position

        self.needle_status = Float32MultiArray()    # Publish [x,y,time, bool: needle detected]
        self.position_table = Float32MultiArray()   # Publish [x,y,z] in table coordinate

        # Subscriber - ROS
        self.sub_get_image = rospy.Subscriber('/image', FloatImage, self.callback_get_image)
        self.sub_xyz = rospy.Subscriber('/robot_position_t', Float32MultiArray, self.callback_get_xyz)

        # Publisher - ROS
        self.pub_xyz = rospy.Publisher('/robot_position_t', Float32MultiArray, queue_size=10)
        self.pub_trigger_send_position = rospy.Publisher('/send_position', Bool, queue_size=10)
        self.pub_needle_detection = rospy.Publisher("needle_detection_t", Float32MultiArray, queue_size=10)

        # Create Objects of other classes
        self.processor = ImageProcessor()      

    def callback_get_image(self, data: FloatImage):
        """Save current image in self.current_image[]"""

        # Reshape the lines of data into a numpy matrix to receive the image
        # Append it to the current_image variable
        image = (np.reshape(data.volume, (data.width, data.height), order='F')).T
        image = (255 * (image - np.min(image)) / np.ptp(image)).astype(int)
        image = np.array(image, dtype=np.uint8)
        self.current_image = image

    def get_image(self):
        """Return current image as Numpy Array"""
        return self.current_image

    def callback_get_xyz(self, rcvd_position_table: Float32MultiArray):
        """ Saved the table position received from robotic group into member variables x_table, y_table, z_table.
            Besides it sets the new_position_received flag to True"""
        self.x_table = rcvd_position_table.data[0]
        self.y_table = rcvd_position_table.data[1]
        self.z_table = rcvd_position_table.data[2]
        self.new_position_received = True

    def publish_xyz(self, x: float, y: float, z: float):
        """Publish x,y,z table coordinates to ros topic /robot_position_t"""
        self.position_table.data = [x, y, z]
        self.pub_xyz.publish(self.position_table)
        rospy.loginfo(self.position_table)

    def get_table_xyz(self):
        """Return x,y,z table coordinates"""
        return self.x_table, self.y_table, self.z_table

    def append_table_xyz(self):
        """Appends latest table position from robotic group to stored_table_position storage array"""
        self.stored_table_positions.append([self.x_table, self.y_table, self.z_table])

    def publish_needle_status(self, x: float, y: float, time, needle_detected: bool):
        """Publish needle_status [x,y,time, bool: needle detected] to /needle_detection_t"""
        self.needle_status.data = [x, y, time, needle_detected]
        self.pub_needle_detection.publish(self.needle_status)
        rospy.loginfo(self.needle_status)

    def append_image_with_coordinate(self):
        """Appends current image to self.stored_images and table position to self.stored_table_positions."""
        self.stored_images.append(self.current_image)
        self.stored_table_positions.append([self.x_table, self.y_table, self.z_table])

    def save_images_with_coordinates(self, path: str):
        """Path: Give Folder at dataname e.g /folder/Dataset1
         Function automatically adds _images.npy and _positions.npy to distinguish between images and positions"""

        np.save(path + "_images.npy", self.stored_images)
        np.save(path + "_positions.npy", self.stored_table_positions)
        rospy.logwarn(f"Saved stored_images with: {len(self.stored_images)} images")
        rospy.logwarn(f"Saved stored_table_position with: {len(self.stored_table_positions)} positions")

    def get_stored_images(self):
        """Returns list of stored images (np arrays) [ [image1], [image2], [image3]....]"""
        return self.stored_images

    def get_stored_table_positions(self):
        """Returns list of stored table positions for each image[ [x1,y1,z1], [x2,y2,z2], [x3,y3,z3]...]"""
        return self.stored_table_positions

    def trigger_send_position(self):
        """ Trigger for send position callback of robotic group, which then publishes the current position
            on /robot_position_t"""
        self.pub_trigger_send_position.publish(True)


if __name__ == '__main__':
    communication_manager = CommunicationManager()
    rospy.spin()
