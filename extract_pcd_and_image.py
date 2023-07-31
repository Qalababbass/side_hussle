import numpy as np
import rosbag
import open3d as o3d
from sensor_msgs import point_cloud2
import cv2
from cv_bridge import CvBridge
import os


def extract_pcd_image(bag):
    start_time = int(bag.get_start_time())
    end_time = int(bag.get_end_time())
 
    time_array=np.linspace(start_time,end_time,num=20,dtype=int)

    bridge = CvBridge()
    
    
     
    for time in time_array:
        ran=False
        
        for topic,msg,t in bag.read_messages(topics=['/ouster_cloud_os1_0/points']):    
            if int(t.to_sec())==time and not ran:
                points=np.array(list(point_cloud2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True)))
                pcd=o3d.geometry.PointCloud()
                pcd.points=o3d.utility.Vector3dVector(points)
                o3d.io.write_point_cloud(os.path.join('lidar_pcd/', "{}.pcd".format(time)), pcd)
                print('Output pointcloud {} as pcd file'.format(time))
                ran=True
    


    for time in time_array:
        ran=False
        
        for topic,msg,t in bag.read_messages(topics=['/zed2i_0/rgb_raw/image_raw_color']):    
            if int(t.to_sec())==time and not ran:            
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv2.imwrite(os.path.join('images_2/', "ground_{}.png".format(time)), cv_img)
                print('Output image {} as .png file '.format(time))
                ran=True




















bag=rosbag.Bag('/home/qalab/DFKI/greenland_height/krone_data/Ground_default_test/recording_2022-12-06-09-53-56_0.bag')
extract_pcd_image(bag)
print('Extraction of pointcloud and images done')