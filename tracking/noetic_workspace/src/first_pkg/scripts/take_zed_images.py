#! /usr/bin/env python

# Resource Used for Inspiration : Stereolabs
# https://github.com/stereolabs/zed-opencv/blob/master/python/zed-opencv.py

import sys
import numpy as np
import pyzed.sl as sl
import cv2
from zed_interfaces.msg import ObjectsStamped
import rospy

help_string = "Saving side by side image based on Object Boundary Box, [q] Quit"
path = "./"

count_save = 0
objects_found = None
PERSON_OBJECT_LABEL = 'Person'

def save_sbs_image(zed, filename) :

    image_sl_left = sl.Mat()
    zed.retrieve_image(image_sl_left, sl.VIEW.LEFT)
    image_cv_left = image_sl_left.get_data()

    image_sl_right = sl.Mat()
    zed.retrieve_image(image_sl_right, sl.VIEW.RIGHT)
    image_cv_right = image_sl_right.get_data()

    sbs_image = np.concatenate((image_cv_left, image_cv_right), axis=1)

    # Get first object in objects that is confidently a person
    object_bounding_box = None
    for object in objects_found:
        if object.confidence > 0.6 and object.label == PERSON_OBJECT_LABEL:
            object_bounding_box = objects_found[0].bounding_box_2d
            break

    if object_bounding_box is not None:
    
        # Define bounding box coordinates
        x_min, y_min = object_bounding_box.corners[0], object_bounding_box.corners[3]
        x_max, y_max = object_bounding_box.corners[1], object_bounding_box.corners[2]

        # Crop image based on the object bounding box
        cropped_image = sbs_image[y_min:y_max, x_min:x_max]

        cv2.imwrite(filename, cropped_image)
        print("Boundary Box Found - Image Saved: " + filename)
    else:
        print("No Boundary Box found - No Image Taken")
    
def object_found_callback(data):
    global objects_found 

    objects_found = data

def process_key_event(zed) :
    global count_save

    save_sbs_image(zed, "ZED_image" + str(count_save) + ".png")
    count_save += 1

def print_help() :
    print(help_string)

def main():

    rospy.Subscriber("zed2i/zed_node/obj_det/objects", ObjectsStamped, object_found_callback)

    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters
    input_type = sl.InputType()
    if len(sys.argv) >= 2 :
        input_type.set_from_svo_file(sys.argv[1])
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.MILLIMETER

    # Open the camera
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS :
        print(repr(err))
        zed.close()
        exit(1)

    # Display help in console
    print_help()

    # Set runtime parameters after opening the camera
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.STANDARD

    # Prepare new image size to retrieve half-resolution images
    image_size = zed.get_camera_information().camera_resolution
    image_size.width = image_size.width /2
    image_size.height = image_size.height /2

    # Declare your sl.Mat matrices
    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

    key = ' '
    while key != 113 :
        err = zed.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS :
            # Retrieve the left image, depth image in the half-resolution
            zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)

            # To recover data from sl.Mat to use it with opencv, use the get_data() method
            # It returns a numpy array that can be used as a matrix with opencv
            image_ocv = image_zed.get_data()

            cv2.imshow("Image", image_ocv)

            key = cv2.waitKey(10)

            process_key_event(zed)

    cv2.destroyAllWindows()
    zed.close()

    print("\nFINISH")

if __name__ == "__main__":
    main()