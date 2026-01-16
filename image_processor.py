#!/usr/bin/env python
# coding: utf-8
import rospy
import cv2 as cv
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from scipy.ndimage import uniform_filter
from scipy.ndimage import variance
import math
import statistics
from skimage import measure
import matplotlib.pyplot as plt


class ImageProcessor():

    def __init__(self):
        # Lucas Kanade Function - Parameter needed
        self.point_selected = False
        self.point = ()
        self.old_points = np.array([[]])
        self.raw_data = []

# Ellipse detection Approaches
# -------------------------------------------------------------------------------------------------------
# The ellipse_char_filter was our final approach for the needle and vessel segmentation used in the presentation
    def calc_hypo(self,a, b):
        """
        simple calculation of triangle a²+b²=c² (used in ellipse_char_filter)

        :param a: a
        :param b: b
        :return: c
        """
        return math.sqrt(a * a + b * b)

    def ellipse_char_filter(self, data, segmentq=3, threshold=150, mindia=5, elips_ratio=4, delete_artifact=False, top_orientation=False):
        """
            takes a grayscale image, binary thrsholdes it, takes every close segment of it.
            Every segment gets approximated by a ellipse, based on the longest diameter and the perpendicular distance to the
            edge of the segment.
            Every "ellipse" than gets passed on if it fulfills criteria.
            :param data: list of np.arrays (images)
            :param segmentq: number of segment that should be analyzed. starting with the "biggest"
            :param threshold: binary threshold max value
            :param mindia: minimal diameter that every segment should have.
            :param elips_ratio: ration between longest radius divided by smaller perpendicular radius
            :return: returns a np array
            """
      
        ellipse_stored_data = []
        for image in data:
            if delete_artifact:
                image[0:125, :] = 0
            if top_orientation:
                image[0:25, :] = 255
            nparray = np.zeros(np.shape(image)).astype(np.uint8)

            # binary threshold image
            (_, binaryim) = cv.threshold(image, threshold, 255, cv.THRESH_BINARY)

            # label segments
            labels_mask = measure.label(binaryim)

            # classify all closed segments in binary image and sort them based on size
            regions = measure.regionprops(labels_mask)
            regions.sort(key=lambda x: x.area, reverse=True)

            # for all segments/props in one image
            if len(regions) > 0:
                for props in regions[:segmentq]:

                    # get center position
                    y0, x0 = props.centroid

                    # get 'orientation' -> angle of longest line zou can draw in segment
                    orientation = props.orientation

                    # calculate point of the smallest distance of segment edge to center
                    x1 = x0 + math.cos(orientation) * 0.5 * props.axis_minor_length
                    y1 = y0 - math.sin(orientation) * 0.5 * props.axis_minor_length

                    # calc point of the longest distance of segment edge to center
                    x2 = x0 - math.sin(orientation) * 0.5 * props.axis_major_length
                    y2 = y0 - math.cos(orientation) * 0.5 * props.axis_major_length

                    # calc radius of ellipse
                    r1 = int(self.calc_hypo(x1 - x0, y1 - y0))
                    r2 = int(self.calc_hypo(x2 - x0, y2 - y0))

                    # minimal radius of the needle image
                    maxr = mindia / 2

                    # conditions for a segment to be a get passed on else: delete point
                    if elips_ratio * r1 >= r2 > maxr:
                        nparray[props.coords[:, 0], props.coords[:, 1]] = 1

            ellipse_stored_data.append(nparray)

        return ellipse_stored_data

    def show_dataset(self, data, interval_time=50, repeat_delay=50):
        """Visualize whole numpy array in one window as animation. Shows raw data animation compared to modified data
        Set time to 0 to slide to next picture with space"""
        print(len(data))

        frames = []  # for storing the generated images

        fig, (ax0, ax1) = plt.subplots(nrows=1, ncols=2, figsize=(5, 5))
        for i in range(0, len(data)):
            raw_image = ax1.imshow(data[i], animated=True, cmap='gray')
            ax0.set_title('Raw Data')
            modified_image = ax0.imshow(self.raw_data[i], animated=True, cmap='gray')
            ax1.set_title('Modified Data')

            frames.append([raw_image, modified_image])

        ani = animation.ArtistAnimation(fig, frames, interval=interval_time, blit=True, repeat_delay=repeat_delay)
        plt.show()

    def show_single_image(self, data, window_name, scale=0.3):
        """data has to be a list of numpy array with len 1. Scale is a factor for the window size"""

        scale = 0.3
        if data[0].ndim == 3:
            height, width, dim = data[0].shape
        elif data[0].ndim == 2:
            height, width = data[0].shape
        height = int(scale * height)
        width = int(scale * width)
        cv.namedWindow(window_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(window_name, width, height)
        cv.imshow(window_name, data[0])
        cv.waitKey(100)


# Hough Transform Approaches
# -------------------------------------------------------------------------------------------------------
# These were approaches when the project goal was still the live-needle tracking.
# Visualizer: Understand what is behind the circular hough transform (self implemented low level version)
# Circular Hough Transform: Used for needle tracking
# Lines Hough Transform: Used for vessel tracking
    def vis_hough_transform(self):
        """ This function was implemented to understand whats inside the opencv circular hough transform function"""
        result = []
        black_img = np.zeros((1000, 900), dtype=np.uint8)
        cimg = cv.cvtColor(black_img, cv.COLOR_GRAY2BGR)
        static_black_img = np.copy(cimg)
        last_circle_img = np.copy(cimg)
        black_img = np.copy(cimg)
        single_circle_on_black = np.copy(black_img)
        cv.circle(cimg, (350, 500), 180, [255, 255, 255], 5)
        cv.circle(cimg, (650, 850), 60, [255, 255, 255], 5)

        temp = np.copy(cimg)
        img = np.copy(cv.cvtColor(temp, cv.COLOR_BGR2GRAY))
        M, N = img.shape

        # Create Window for Image and Hough Transform
        scale = 0.3
        height, width = img.shape
        height = int(scale * height)
        width = int(scale * width)
        cv.namedWindow("Image", cv.WINDOW_NORMAL)
        cv.resizeWindow('Image', width, height)
        cv.namedWindow("Hough Transform", cv.WINDOW_NORMAL)
        cv.resizeWindow('Hough Transform', width, height)

        for m in range(200, M, 20):
            for n in range(0, N, 12):
                # Iterate through every pixel
                temp = np.copy(cimg)
                cv.circle(cimg, (n, m), 5, (0, 0, 255), -1)

                # cimg[m][n] = [0, 0, 255]
                # If pixel is not zero draw circle of size we are looking for

                if img[m][n] > 0:
                    black_img = black_img + last_circle_img
                    cv.circle(single_circle_on_black, (n, m), 180, (0, 0, 45), 2)

                black_img = black_img + single_circle_on_black
                result.append(black_img)
                # combined_image = np.hstack((cimg, black_image))
                cv.imshow('Image', cimg)
                cv.imshow('Hough Transform', black_img)
                cv.waitKey(1)
                cimg = np.copy(temp)
                single_circle_on_black = np.copy(static_black_img)

    def circular_houg_transform(self, data, min_radius=11, max_radius=28, param_1=50, param_2=6):

        image_array = []
        for img in data:

            cimg = cv.cvtColor(img, cv.COLOR_GRAY2BGR)

            circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, 1, 20,
                                      param1=param_1, param2=param_2, minRadius=min_radius, maxRadius=max_radius)

            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    # draw the outer circle
                    print(f"x={i[0]},y={i[1]}")
                    cv.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 5)
                    # draw the center of the circle
                    cv.circle(cimg, (i[0], i[1]), 2, (0, 255, 0), 3)
            image_array.append(cimg)
        return image_array

    def line_hough_transform(self, data, draw_circles: bool = False):
        image_array = []
        x_axis, y_axis = data[0].shape
        for image in data:
            lines = cv.HoughLinesP(image, 1, np.pi / 180, threshold=150, minLineLength=50, maxLineGap=120)

            blank_image = np.zeros((x_axis, y_axis), np.uint8)

            # Draw lines on the image
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv.line(blank_image, (x1, y1), (x2, y2), (255, 0, 0), 3)
            image_array.append(blank_image)
        return image_array


# Lucas Kanade Optical Flow Approach
# -------------------------------------------------------------------------------------------------------
# select_points: Used to select a point by mouseclicking
# lucas_kanade: Optical Flow algorith to track the selected points according to parameter set.
    def select_point(self, event, x, y, flags, params):
        """ Mouse Click Event - Helper Function For Lucas Kanade Alogrithm to select point by mouse.
            is used in lucas_kanade"""
        if event == cv.EVENT_LBUTTONDOWN:
            x = 8 * x
            y = 8 * y
            self.point = (x, y)
            self.point_selected = True
            print(self.point_selected)
            self.old_points = np.array([[x, y]], dtype=np.float32)

    def lucas_kanade(self, data, squared_window_size=75):
        """ Optical Flow Based Algorithm - Creates area of interest around mouse selected point. Area of interest is
            defined by squared_window_size. Lucas Kanade shows data in itself and return no data right now"""
        # Lucas kanade params
        lk_params = dict(winSize=(squared_window_size, squared_window_size),
                         maxLevel=4,
                         criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

        cv.namedWindow("Data", cv.WINDOW_NORMAL)
        cv.setMouseCallback("Data", self.select_point)

        old_image = data[0]

        while not self.point_selected:
            for image in data:
                if self.point_selected is True:
                    new_points, status, error = cv.calcOpticalFlowPyrLK(old_image, image, self.old_points, None,
                                                                        **lk_params)
                    old_image = image.copy()
                    self.old_points = new_points
                    print(self.old_points)

                    x, y = new_points.ravel()

                    cv.circle(image, (int(x), int(y)), 100, (255, 255, 255), 10)

                first_level = cv.pyrDown(image)
                second_level = cv.pyrDown(first_level)
                third_level = cv.pyrDown(second_level)

                y_axis, x_axis = image.shape
                scale = 2
                x_axis = int(x_axis / scale)
                y_axis = int(y_axis / scale)
                cv.resizeWindow('Data', x_axis, y_axis)
                cv.imshow("Data", third_level)
                key = cv.waitKey(0)

                if key == 27:
                    break
                time.sleep(0.5)

        cv.destroyAllWindows()


# Basic IP Methods
    def flip_array_lr(self, data):
        """Flips array from left to right. E.g. when ultrasonic head is rotated 180 degree."""
        stored_images = []
        for array in data:
            array = np.fliplr(array)
            stored_images.append(array)
        return stored_images

    def resize_image(self, img, scaling_factor):
        """ Resizes Image according to scaling factor. 1 = original size"""
        M, N = img.shape
        new_height = int(M / scaling_factor)
        new_width = int(N / scaling_factor)
        image = cv.resize(img, (new_height, new_width))
        return image

    def sobel_on_array(self, data):
        """ Edge detection based on sobel operator"""
        image_array = []
        for image in data:
            sobely = cv.Sobel(image, cv.CV_8UC1, 0, 1, ksize=5)
            image_array.append(sobely)
        return image_array

    def threshold_filter(self, data, threshold_value=200, max_value=250, delete_artifact: bool = False,
                         blur: bool = True, blur_kernel_size=63):
        """ Thresholds the whole list of numpy array. Set threshold_value and max_value for everything above it.
            Set bool True or False to blur the image before thresholding"""

        image_array = []
        for image in data:
            # Keep in mind to deepcopy(image_array) otherwise it is just a reference and you will debug alot
            if delete_artifact:
                image[0:125, :] = 0
            if blur:
                blurred = cv.GaussianBlur(image, (blur_kernel_size, blur_kernel_size), 0)
                ret, tframe = cv.threshold(blurred, threshold_value, max_value, cv.THRESH_BINARY)
            else:
                ret, tframe = cv.threshold(image, threshold_value, max_value, cv.THRESH_BINARY)

            image_array.append(tframe)

        return image_array
  
    def convert_dataset(self, data):
        converted_data = []
        for image in data:
            new_image = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
            converted_data.append(new_image)
        return converted_data





if __name__ == "__main__":
    print("Test")
    image_processor = ImageProcessor()
    image_processor.main()
