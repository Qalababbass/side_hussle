import open3d as o3d
import numpy as np
import sys
import os
import rosbag
from sensor_msgs import point_cloud2
import cv2
import matplotlib.pyplot as plt
from PIL import Image



############################# Reading pixel values in image based on their coordinates #######################

# img=cv2.imread('Tools_Merge_Image_PointCloud/img/000003.png')
# (h,w)=img.shape[:2]

# (b,g,r)=img[10,15] # pixel located at x=15,y=10
# #(Cx,Cy)=(w//2,h//2)
# #tl=img[0:Cy,0:Cx]

# #img[0:Cy,0:Cx]=(0,255,0)

# print('Pixel at (15,10) - Red {}, Green {}, Blue {} '.format(r,g,b))
# cv2.imshow('Top left Corner' , img)

# K=cv2.waitKey(0)
################################################################################################













################################# Creating colored pcd from RGBD image ######################################

height,width,Fx,Fy,Cx,Cy=540,960,536.0599,536.0449,494.54,260.9169

color_raw = o3d.io.read_image("images/recording_2022-10-18-12-20-14_0_rgb.png")
color_np=np.asarray(color_raw)
print('RGB Image datatype : {}'.format(color_np.dtype))
depth_raw = o3d.io.read_image("images/recording_2022-10-18-12-20-14_0_depth.png")
depth_np=np.asarray(depth_raw)
print('Depth image datatype : {}'.format(depth_np.dtype))

rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw,convert_rgb_to_intensity=False)

print('RGBD Image data : {}'.format(rgbd_image))

intrinsic = o3d.camera.PinholeCameraIntrinsic(width,height,Fx,Fy,Cx,Cy)
intrinsic.intrinsic_matrix=[[Fx, 0, Cx], [0, Fy, Cy], [0, 0, 1]]

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,intrinsic)
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
o3d.visualization.draw_geometries([pcd])

plt.subplot(1, 2, 1)
plt.title('ground rbg image')
plt.imshow(rgbd_image.color)
plt.subplot(1, 2, 2)
plt.title('ground depth image')
plt.imshow(rgbd_image.depth)
plt.show()


############################################################################################################################

