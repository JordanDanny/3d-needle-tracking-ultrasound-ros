import numpy as np
import open3d as o3d



class VolumeCreator:

    def __init__(self):
        self.probe_width = 38.1                 # for scanaxis x = positiv direction, scanaxis y = negativ direction
        self.phantom_height = 100               # Height of ultrasonic image e.g. phantom height (just approximation)


    def images_to_coordinates(self, image_data, position_data, scan_axis, epsilon=0.1):
        """
        Maps pixels from 2D images to 3D world coordinates based on the robot's position.
        """

        stored_coordinate_list = []
        calculate_new_width_height_matrix = True  # Calculate first image plane coordinate matrix for first scanline

        # Set scan axis according to user input
        if scan_axis == 'x':
            scan_axis = 0
            shift_axis = 1
        elif scan_axis == 'y':
            scan_axis = 1
            shift_axis = 0


        # Calculate pixel to coordinate matrix
        for image in range(len(image_data)):
            if calculate_new_width_height_matrix:
                # Calculate width, depth matrix to link pixel coordinate to table coordinate according to chosen scanline
                if scan_axis == 0:
                    # - self.probe_width negativ y direction
                    # - self.phantom_height because negative z direction
                    y, z = self.create_pixel_to_coordinate_matrix(image_data[image], position_data[image],
                                                                  -self.probe_width, -self.phantom_height,
                                                                  shift_axis=shift_axis)
    
                elif scan_axis == 1:
                    x, z = self.create_pixel_to_coordinate_matrix(image_data[image], position_data[image],
                                                                  self.probe_width, -self.phantom_height, shift_axis=shift_axis)
                 
            if image < len(image_data) - 1:
                if abs(position_data[image][shift_axis] - position_data[image + 1][shift_axis]) > epsilon:
                    calculate_new_width_height_matrix = True

            # Returns list of non zero elements in matrix: Basically all white pixels (ideal case: Needle and vessel)
            list_non_zero_indices = np.transpose(np.nonzero(image_data[image]))  # [ [row,column], [row,column]....]

            # Save table coordinates of non zero elements of each image in a storage list
            for index in list_non_zero_indices:
                if scan_axis == 0:
                    stored_coordinate_list.append([position_data[image][scan_axis], y[index[0]][index[1]], z[index[0]][index[1]]])
                elif scan_axis == 1:
                    stored_coordinate_list.append([x[index[0]][index[1]], position_data[image][scan_axis], z[index[0]][index[1]]])


        return stored_coordinate_list

    def calculate_3D_volume(self, image_data, position_data, scan_axis):
        """ Input image data (list of numpy arrays) and position data (list of (x,y,z) arrays) of same length.
            Image data should be preprocessed e.g thresholded, blurred etc.
            This function filters out data with shifts in y-directions between multiple scanlines and then creates a
            pointcloud out of all remaining images. These are true to scale. You can pick two points in the 3D volume
            by [shift + left click] and close the windows with 'q'.
            As a return value you get the distance between those points y-axis"""
        
        # Filter Position Data: Discard shift movement between scanlines and double data
        filtered_image_data, filtered_position_data = self.filter_position_data(image_data, position_data, scan_axis=scan_axis)

        # Produce x,y,z coordinate for all images (multiple scanlines included)
        stored_coordinate_list = self.images_to_coordinates(filtered_image_data, filtered_position_data, scan_axis=scan_axis)

        # Create Pointcloud
        pcd = self.create_pointcloud(stored_coordinate_list)

        # Pick points and calculate distance in y-direction
        points_index = self.pick_points(pcd)

        # Display selected points by index (not in-build open3d)
        print(np.asarray(pcd.points[points_index[0][0]]))
        print(np.asarray(pcd.points[points_index[0][1]]))

        # Calculate distance in y component of selected points
        point1 = np.asarray(pcd.points[points_index[0][0]])
        point2 = np.asarray(pcd.points[points_index[0][1]])
        # x = 0, y = 1, z = 2
        difference_abs = abs(point1[1] - point2[1])
        print(difference_abs)
        return difference_abs

    def filter_position_data(self, image_data, position_data, scan_axis, epsilon=0.1):
        """ Filter position data to discard shift between scanlines. Set epsilon for biggest difference allowed between
            y-component between adjacent position data --> Detect shifts between scanlines
            Also detects double data in x direction (same x position mutliple times) and filters it out."""
        #print(f"Scan axis in filterposition data_dynamisch beginn: {scan_axis}")
        if scan_axis == 'x':
            scan_axis = 0
            shift_axis = 1
            #print(f"Scan axis wenn x achse: {scan_axis}")
        elif scan_axis == 'y':
            scan_axis = 1
            shift_axis = 0
            #print(f"Scan axis wenn y achse: {scan_axis}")

        # 1. for -> Loop through all position data e.g [x1,y1,z1]....[x2,y2,z2]
        # 2. if  -> Checking if there is still a "neighbour position data to compare"
        # 3. if  -> Calculating the difference in y value of neighbour position data and check if smaller than epsilon
        # 4.        keep position data and image data if smaller episode: bigger than epsilon --> Shift in Y direction -> ignore data
        filtered_position_data = []
        filtered_image_data = []
        for pos_index in range(len(position_data)):
            if pos_index < len(position_data) - 1:
                # Difference between e.g y value of pos1 to y value of pos 2: set shift_filter_coordinate to chose axis
                neighbour_difference = abs(position_data[pos_index][shift_axis] - position_data[pos_index + 1][shift_axis])
                # Double_data_filter checks for x1 = x2 during one scan line: If x does not change, we got the same
                # position twice, due to some server position request problem. Double data = Just take the 2nd one ignore 1st
                double_data_filter = abs(position_data[pos_index][scan_axis] - position_data[pos_index + 1][scan_axis])

                if neighbour_difference < epsilon and double_data_filter != 0:
                    # Append position and image when no double data and no shift detected -> basically extract scanline
                    filtered_position_data.append(position_data[pos_index])
                    filtered_image_data.append(image_data[pos_index])

        return filtered_image_data, filtered_position_data

    def create_pixel_to_coordinate_matrix(self, image, position, probe_width, phantom_height, shift_axis):
        """ Takes the image width,height in pixel, the position of the scanline, the width of
            the ultrasonic probe in mm and the height of phantom in mm (z-plane) to calculate two matrices
            y and z or x and z which gives e.g. y and z value for each pixel. E.g y[10][15] = y value at pixel row:10, column:15 """

        # Take height and width in pixel coordinates necessary for the np.meshgrid function to calculate y and z matrix
        height_px_image, width_px_image = image.shape

        # Calculate stepsize for meshgrid function to distribute probe width and phantom height homogenously over all
        # pixels
        stepsize_width = probe_width / width_px_image
        stepsize_height = phantom_height / height_px_image

        # Top Left Corner Y Value as reference point: probe width is distributed from top left corner to top right
        # corner. Z value -> topleft corner = 0 since we do not focus on the real height
        width_origin = position[shift_axis]

        # Create two matrices y and z: e.g y[height,width] shows the y value at this pixel-position. Same for z matrix
        width_matrix, z_ = np.mgrid[width_origin:(width_origin + probe_width):stepsize_width, 0:phantom_height:stepsize_height]
        width_matrix = width_matrix.T
        z_ = z_.T

        return width_matrix, z_

    def pick_points(self, pcd):
        print("1) Pick two points you want to calculate the distance of [shift + left click]")
        print("   Undo point picking [shift + right click]")
        print("2) press q when you picked your points")

        # Create Editor Windows With Loaded Pointcloud
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(pcd)

        # Start Picking Two Points
        vis.run()
        vis.destroy_window()

        # Return indices of picked points in Pointcloud
        return vis.get_picked_points(), vis

    def create_pointcloud(self, coordinate_list):
        """ Takes a list of (x,y,z) coordinate and converts them into a pointcloud and returns pcd"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(coordinate_list)

        print(f"{pcd}")
        return pcd



if __name__ == '__main__':
    volume_creator = VolumeCreator()