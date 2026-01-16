

from VolumeCreator import VolumeCreator
from ImageProcessor import ImageProcessor
import numpy as np
import rospy



# Safety check: Try to import hardware interfaces. 
# If running on a PC without the robot/motor files
try:
    from motor_interface import MotorInterface, BoardInterface
    HARDWARE_AVAILABLE = True
except ImportError:
    print("Warning: 'motor_interface.py' not found. Hardware control will be disabled.")
    HARDWARE_AVAILABLE = False

def main():
    
    # Initialize ROS Node (Required for logging and communication)
    rospy.init_node('processor_3D_volume_cr', anonymous=True)

    # Instantiate processing classes
    volume_creator = VolumeCreator()
    image_processor = ImageProcessor()
    # Data Loading Path
    path = r"D:\data"  # Set folder and dataset number: _images, _position is set automatically

    try:
        # Load image and position data
        stored_images = np.load(path + "_images.npy")
        stored_table_positions = np.load(path + "_positions.npy")
        rospy.loginfo(f"Loaded data from {path}")
    except FileNotFoundError:
        rospy.logerr(f"Could not find data files at: {path}")
        return

    # Set raw_data for comparison after modifying: self.showdataset()
    image_processor.raw_data = stored_images


    # ---------------------------------------------------------
    # 1. Segmentation / Preprocessing
    # ---------------------------------------------------------
    rospy.logwarn("Segmentation of Image data has started")

    # Use Ellipse Filter for Segmentation of Vessel and Needle
    ellipse_stored_images = image_processor.ellipse_char_filter(stored_images[:], threshold=200, mindia=20,
                                                                elips_ratio=1000,
                                                                segmentq=3, delete_artifact=True, top_orientation=True)
     
    # Visualize raw data in comparison to modified data: Make sure preprocessing was done right
    rospy.logwarn("Visualizing Dataset")
    image_processor.show_dataset(ellipse_stored_images, interval_time=50, repeat_delay=50)


    # ---------------------------------------------------------
    # 2. 3D Volume Creation
    # ---------------------------------------------------------
    scan_axis = input("Please select the scan axis according to table coordinates. Type in 'x' or 'y' \n")
    rospy.logwarn("3D Volume Creation started")

    # Calculate volume and measure distance between two selected points
    distance = volume_creator.calculate_3D_volume(ellipse_stored_images, stored_table_positions,
                                                  scan_axis=scan_axis)


    # ---------------------------------------------------------
    # 3. Robot / Motor Control (Hardware Dependent)
    # ---------------------------------------------------------
    print(f"Calculated distance: {distance}mm")
    if not HARDWARE_AVAILABLE:
        print("Hardware interface not available. Skipping motor movement.")
        return
    
    print(f"Do you want to move the needle by distance: {distance}mm?")
    answer = input("Type in 'y' , 'n' or 'custom'\n")

    if answer == 'y':
        print("Needle is moving now. Be careful")
        board_interface = BoardInterface('/dev/ttyACM0')
        motor_needle = MotorInterface("needle", "N", 6100, board_interface)
        motor_phantom = MotorInterface("phantom", "P", 2000, board_interface)
        motor_needle.move_mm(distance)

    elif answer == 'custom':
        distance = input("Type in the distance you want to move in [mm] without units.\n")
        print(f"Needle is moving by distance: {distance}mm. Be careful")
        board_interface = BoardInterface('/dev/ttyACM0')
        motor_needle = MotorInterface("needle", "N", 6100, board_interface)
        motor_phantom = MotorInterface("phantom", "P", 2000, board_interface)
        motor_needle.move_mm(distance)


if __name__ == '__main__':
    main()
