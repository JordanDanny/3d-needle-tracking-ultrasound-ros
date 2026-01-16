import rospy
import numpy as np
from supra_msgs.msg import FloatImage
from CommunicationManager import CommunicationManager

# This script has the task of saving images and position data into an array. For this task the position data needs to be
# requested by the robotic group by triggering "/send_position" ros topic. Then the published position data is read out
# from the "/robot_position_t" and appended to a "stored_table_positions" array list. Parallel we append the corresponding
# image to the "stored_images" array list. Last step is saving both list of arrays to be ready to use.

stored_data = []
image_counter = 0
# Choose Path for Saving: e.g. dataset001 get split to dataset001_images.npy and dataset001_positions.npy
communication_manager.path_data = "/home/tuhh/catkin_ws/src/image_processor/scripts/paperdata/dataset001"  

def reshape_stored_data(data_list):
    """Reshape the list of serial data stream (multiple images) and return list of reshaped arrays (images)"""
    # Make sure stored images is empty, before appending reshaped images
    communication_manager.stored_images = []

    # Reshape from serial data stream to image array and appending it to stored_images, which is saved later
    # on with position data
    for serial_data in data_list:
        image = (np.reshape(serial_data.volume, (serial_data.width, serial_data.height), order='F')).T
        image = (255 * (image - np.min(image)) / np.ptp(image)).astype(int)
        image = np.array(image, dtype=np.uint8)

        # Append reshaped image to stored images, which is saved at the end
        communication_manager.stored_images.append(image)

def callback_us_image(data):

    if not communication_manager.already_position_requested:
        # Trigger Callback Function of Robotic Group: send position data on "/robot_position_t
        communication_manager.trigger_send_position()

    if communication_manager.new_position_received:
        # Append Position of image to self.stored_table_positions
        communication_manager.append_table_xyz()
        communication_manager.new_position_received = False
        communication_manager.already_position_requested = False

        # Append current image as serial data stream: reshaping happens later on due to computation time
        stored_data.append(data)
        image_counter += 1
        rospy.logwarn(f"images and positions saved:: {image_counter}")

def listener():
    # Initialize ROS Node
    rospy.init_node('processor_scanline_saver', anonymous=True)
    rospy.logwarn("Scan Line saver has been started")

    # Create SUPRA Subscriber to receive US Images
    image_sub = rospy.Subscriber('/image', FloatImage, callback_us_image)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    communication_manager = CommunicationManager()
    listener()

    # Reshapes list of data list (list of serial data streams) and internally append it to self.stored_images
    # which is saved by self.save_images_with_coordinates
    reshape_stored_data(stored_data)

    # Save stored_images and stored_table_positions list of arrays as .npy file
    communication_manager.save_images_with_coordinates(communication_manager.path_data)
    rospy.logwarn("Scan Line saver has been terminated")
