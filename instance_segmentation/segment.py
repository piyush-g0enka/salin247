#!/usr/bin/env python3

#############################################################################################
# File : segment.py
# Function : File contains a class Segment that is used to perform instance segmentation
#            on an inout image.
#############################################################################################


import cv2
import numpy as np

#############################################################################################
# Name : Segment
# Function : Segment is a class that comprises of functions which create masks
#            on distinct crop rows
#############################################################################################


class Segment:

    def __init__(self, filename):

        self.original_image = cv2.imread(filename)

    #############################################################################################
    # Name : create_binary_image
    # Function : Creates white-black image from an rgb image with white pixels being applied in
    #            pixels that satisfy a specific condition
    #############################################################################################

    def create_binary_image(self, image):

        height, width, _ = image.shape
        binary_image = np.zeros((height, width, 1), dtype=np.uint8)

        for y in range(image.shape[0]):
            for x in range(image.shape[1]):
                # Get the RGB values of the pixel
                b, g, r = image[y, x]

                # Check if pixel values represent a crop row
                # Conditions are extracted from close examination of image pixels

                if r > 100 and g > 100 and b > 100 and (r >= g) and (g/b < 1.2):
                    # Make the pixel white
                    binary_image[y, x] = [255]

                else:
                    # Make the pixel black
                    binary_image[y, x] = [0]

        return binary_image

    #############################################################################################
    # Name : thinning_image
    # Function : Thins the white pixels
    #############################################################################################

    def thinning_image(self, image):

        # Thinning
        thin_image = cv2.ximgproc.thinning(image)
        return thin_image

    #############################################################################################
    # Name : extract_edges
    # Function : Extracts edges from an image with direction of edge specified as either
    #            'vertical' or 'horizontal'
    #############################################################################################
    def extract_edges(self, thin_image, direction):

        # Get image dimensions
        image_height, image_width = thin_image.shape

        # Define step sizes
        if direction == "horizontal":
            step_width = 20
            step_height = 3
        else:
            step_width = 3
            step_height = 20

        threshold_value = 18

        # Create a copy of the thinned image to make changes
        edges_image = thin_image.copy()

        # Iterate over the image with the specified step sizes
        for y in range(0, image_height, step_height):
            for x in range(0, image_width, step_width):
                # Calculate the region of interest for this step
                roi = thin_image[y:min(
                    y+step_height, image_height), x:min(x+step_width, image_width)]

                # Calculate the sum of pixel values in the ROI
                sum_pixels = np.sum(roi)

                # If the sum of pixel values is less than the threshold, set all pixels in that step to black
                if sum_pixels/255 < threshold_value:
                    edges_image[y:min(y+step_height, image_height),
                                x:min(x+step_width, image_width)] = 0

        return edges_image

    #############################################################################################
    # Name : remove_noise
    # Function : removes small collection of white pixels from an image
    #############################################################################################
    def remove_noise(self, image):

        image_height, image_width = image.shape

        # step sizes
        step_width = 100
        step_height = 100

        threshold_value = 100

        filtered_image = image.copy()

        # Iterate over the image with the specified step sizes
        for y in range(0, image_height, step_height):
            for x in range(0, image_width, step_width):
                # Calculate the region of interest for this step
                roi = image[y:min(y+step_height, image_height),
                            x:min(x+step_width, image_width)]

                # Calculate the sum of pixel values in the ROI
                sum_pixels = np.sum(roi)

                # If the sum of pixel values is less than the threshold, set all pixels in that step to black
                if sum_pixels/255 < threshold_value:
                    filtered_image[y:min(
                        y+step_height, image_height), x:min(x+step_width, image_width)] = 0

        return filtered_image

    #############################################################################################
    # Name : thicken_image
    # Function : Dilates an image thereby making the white portions of the image thicker
    #############################################################################################

    def thicken_image(self, image, kernel_size=50):

        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        # Perform dilation on the binary image
        thickened_image = cv2.dilate(image, kernel, iterations=1)

        return thickened_image

    #############################################################################################
    # Name : subtract_image
    # Function : Subtracts white pixels of an image from a list of image masks
    #############################################################################################

    def subtract_image(self, image,  image_masks):

        subtracted_image = image.copy()
        for mask in image_masks:
            subtracted_image = cv2.subtract(subtracted_image, mask)
        return subtracted_image

    #############################################################################################
    # Name : apply_mask
    # Function : applies the red, blue and green masks to an image
    #############################################################################################

    def apply_mask(self, bgr_image, masks):
        image = bgr_image.copy()
        for y in range(image.shape[0]):
            for x in range(image.shape[1]):
                # Get the RGB values of the pixel
                b, g, r = image[y, x]

                # Check if all RGB values are above 100
                if r > 100 and g > 100 and b > 100 and (r >= g) and (g/b < 1.2):
                    # Make the pixel white

                    if masks[0][y, x] == 255:
                        image[y, x] = (0, 255, 0)

                    if masks[1][y, x] == 255:
                        image[y, x] = (0, 0, 255)

                    if masks[2][y, x] == 255:
                        image[y, x] = (255, 0, 0)

        return image

    #############################################################################################
    # Name : execute
    # Function : executes the image processing pipeline
    #############################################################################################

    def execute(self):

        print("Creating Binary image...")
        self.binary_image = self.create_binary_image(self.original_image)

        print("Thinning the Binary image...")
        self.thin_image = self.thinning_image(self.binary_image)

        print("Extracting vertical edges...")
        self.vertical_edges = self.extract_edges(self.thin_image, "vertical")

        print("De-noising vertical edges...")
        self.filtered_vertical_edges = self.remove_noise(self.vertical_edges)

        print("Extracting horizontal edges...")
        self.horizontal_edges = self.extract_edges(
            self.thin_image, "horizontal")

        print("De-noising horizontal edges...")
        self.filtered_horizontal_edges = self.remove_noise(
            self.horizontal_edges)

        print("Creating vertical edges mask...")
        self.vertical_edges_mask = self.thicken_image(
            self.filtered_vertical_edges)

        print("Creating horizontal edges mask...")
        self.horizontal_edges_mask = self.thicken_image(
            self.filtered_horizontal_edges)

        print("Extracting diagonal edges...")
        self.diagonal_edges = self.subtract_image(
            self.thin_image, [self.vertical_edges_mask, self.horizontal_edges_mask])

        print("De-noising diagonal edges...")
        self.filtered_diagonal_edges = self.remove_noise(self.diagonal_edges)

        print("Creating diagonal edges mask...")
        self.diagonal_edges_mask = self.thicken_image(
            self.filtered_diagonal_edges, kernel_size=15)

        print("Applying masks to the original image...")
        self.final_image = self.apply_mask(self.original_image, [
                                           self.diagonal_edges_mask, self.vertical_edges_mask, self.horizontal_edges_mask])

        print("Final image created as final_image.jpg")
        cv2.imwrite("final_image"+".jpg", self.final_image)


if __name__ == "__main__":

    Segment("field.jpg").execute()
