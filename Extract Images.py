'''
Script is used to extract the images from camera for the same timestamp at which the pointcloud was extracted.
THe input is a rosbag file and output is a png image file
'''


import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
   # """Extract a folder of images from a rosbag.
   # """
   # parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
   # parser.add_argument("bag_file", help="Input ROS bag.")
   # parser.add_argument("output_dir", help="Output directory.")
   # parser.add_argument("image_topic", help="Image topic.")
#
   # args = parser.parse_args()

   # print ("Extract images from %s on topic %s into %s" % (args.bag_file,
   #                                                       args.image_topic, args.output_dir))



    bag = rosbag.Bag('rosbag_file.bag', "r") # Give the rosbag path here
    bridge = CvBridge()
    count = 0
    ran=False
    for topic, msg, t in bag.read_messages(topics=['/image_topic_name']): # Insert the image topic name here from which image is to be extracted
        if t.secs==1666088444 and not ran:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            cv2.imwrite(os.path.join('images/', "image_file.png"), cv_img)
            ran=True
            #print ("Wrote image " % count)

        count += 1


    return

if __name__ == '__main__':
    main()