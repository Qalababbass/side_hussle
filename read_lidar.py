import rosbag
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import copy



def write_rosbag(bag):
    '''
    Function used to write a rosbag from given input rosbag file. The resulting rosbag only contains a single point_cloud and tf messages of certain
    time stamp given in seconds
    '''
    bag3=rosbag.Bag('greenlanddata/recording_2022-10-18-13-37-27_0/end_1666093110.bag','w')
    ran=False
    for topic,msg,t in bag.read_messages(topics=['/ouster_cloud_os1_0/points','/tf']):
        if t.secs==1666093110:
            if topic=='/ouster_cloud_os1_0/points' and not ran:
                bag3.write(topic,msg)
                ran=True
            elif topic=='/tf':
                bag3.write(topic,msg)
    bag3.close()


def write_rosbag_ground(bag):
    '''
    This function takes in rosbag file as input and use this rosbag to write another rosbag file containing only the /point_cloud_topic_name and /tf msg
    at specified t.secs timestamp value.
    '''
    bag3=rosbag.Bag('new_rosbag_file.bag','w')
    ran=False
    for topic,msg,t in bag.read_messages(topics=['/point_cloud_topic_name','/tf']):
        if t.secs==1670316860:
            if topic=='/point_cloud_topic_name' and not ran:
                bag3.write(topic,msg)
                ran=True
            elif topic=='/tf':
                bag3.write(topic,msg)
    bag3.close()

def extract_lidar_point(bag):
    '''
    This function takes in rosbag as input file and reads the pointcloud msg from /point_cloud_topic_name at specified t.secs timestamp. It extracts the x,y,z field from msg data
    and save it into a numpy array.
    It then saves these extracted points into a .npy format
    '''

    ran=False
    for topic,msg,t in bag.read_messages(topics='/point_cloud_topic_name'):
        if t.secs==1666091281:
            if topic=='/point_cloud_topic_name' and not ran:
                points=np.array(list(point_cloud2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True)))
    with open('test.npy','wb') as f:
        np.save(f,points)


def pc_to_csv(bag):                                                                                     
    '''
    This function is used to create a csv file from a pointcloud message type. THe input is a rosbag file, and the function returns extracted
    properties of pointcloud in a csv format at the specified timestamp
    '''
    ran=False
    for topic,msg,t in bag.read_messages(topics='/point_cloud_topic_name'):
        if t.secs==1666088434:
            if topic=='/point_cloud_topic_name' and not ran:
                points=np.array(list(point_cloud2.read_points(msg, field_names = ("x", "y", "z","intensity","reflectivity","range"), skip_nans=True)))
    with open('./greenland_2_checking.csv', 'w') as f:
        for point in points:
            f.write(','.join(str(x) for x in point) + '\n')

def pointcloud_to_pcd(bag):
    '''
    This function takes in rosbag file as input parameter. It then reads the pointcloud_topic_name msg from rosbag and get the specified x,y,z,rgb fields
    from pointcloud msg and save it in a numpy array.
    It then converts the array to Open3d pontcloud object and save it into a .pcd file format.
    '''

    ran=False
    for topic,msg,t in bag.read_messages(topics='/point_cloud_topic_name'):
        if t.secs==1686230969:
            if topic=='/point_cloud_topic_name' and not ran:
                points=np.array(list(point_cloud2.read_points(msg, field_names = ("x", "y", "z","rgb"), skip_nans=True)))
    pcd=o3d.geometry.PointCloud()
    pcd.points=o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud("pointcloud_data.pcd", pcd)
#roi_width = 10.0 # 10 cm
# roi_height = 10.0 # 10 cm
# roi_depth=100.0
# roi_center = [0.0, 0.0, 0.0] # ROI center coordinates

# # Create Open3D point cloud object from numpy array
# pointcloud=o3d.io.read_point_cloud('./greenland.pcd')
# o3d.visualization.draw_geometries([pointcloud])
# pointcloud_r=copy.deepcopy(pointcloud)
# R = pointcloud.get_rotation_matrix_from_xyz((0,np.pi/4, 0))
# pointcloud_r.rotate(R, center=(0, 0, 0))

# # Define ROI region as a bounding box
# xmin = roi_center[0] - roi_width/2.0
# xmax = roi_center[0] + roi_width/2.0
# ymin = roi_center[1] - roi_height/3.0  #original 2.0
# ymax = roi_center[1] + roi_height/3.0 # original 2.0
# zmin = roi_center[2] - roi_depth/2.0
# zmax = roi_center[2] + roi_depth/2.0 # adjust this value to include desired height range

# # Extract ROI points using a crop box filter
# bbox = o3d.geometry.AxisAlignedBoundingBox(
#     min_bound=[xmin, ymin, zmin],
#     max_bound=[xmax, ymax, zmax]
# )

# #greenaland_2_ipad points min_bound=[4.205, -1.050, -0.731] max_bound=[4.385, 1.448, -0.557]
# #greenland_2_ipad_2 points min bound=[4.205, -1.050, -0.731] and max bound = [4.399, 1.448, -0.539]
# #greenland_2_ipad_3 points min_bound=[4.183, -1.050, -0.731] max_bound=[4.399, 1.448, -0.539]        

# # greenland_ipad.pcd min_bound=[4.205, -1.050, -0.731] max_bound=[4.385, 1.448, -0.557] these are the values obtained and written in ipad
# # greenland_ipad_2.pcd min_bound=[4.205, -1.050, -0.731] max_bound=[4.399, 1.448, -0.539]
# # greenland_ipad_3.pcd min_bound=[4.183, -1.050, -0.731] max_bound=[4.399, 1.448, -0.539]
# # last set values min_bound=[3.0, -1.050, -0.5] max_bound=[5.0, 1.448, -0.195]


# bbox_2 = o3d.geometry.AxisAlignedBoundingBox(
#     min_bound=[2.9, -0.5, -4],
#     max_bound=[3.9, 0.5, -2]
# )


# pointcloud_roi = pointcloud_r.crop(bbox_2)
# print('Shape of points ',np.asarray(pointcloud.points).shape)
# #print(np.asarray(pointcloud.points[3358:3372]))
# print('Shape of points of roi ',np.asarray(pointcloud_roi.points).shape)
# o3d.visualization.draw_geometries([pointcloud_roi])

# # Save ROI point cloud to PLY file
# o3d.io.write_point_cloud("greenland_ccw_crop.pcd", pointcloud_roi)
# o3d.io.write_point_cloud("greenland_ccw.pcd", pointcloud_r)

# pointcloud_roi_rev=copy.deepcopy(pointcloud_roi)
# R_2 = pointcloud_roi.get_rotation_matrix_from_xyz((0,-np.pi/4, 0))
# pointcloud_roi_rev.rotate(R_2, center=(0, 0, 0))
# print('Shape of pointcloud_reverse points {}'.format(np.asarray(pointcloud_roi_rev.points).shape))
# o3d.io.write_point_cloud("greenland_2_ccw_crop_reversed.pcd", pointcloud_roi_rev)

def pc_visualization():
    '''
    This function uses open3d for visualizing pointcloud saved in a .pcd format in python
    '''

    pc_pcd=o3d.io.read_point_cloud('rgb_pcd_and_rosbag/greenland_rgb.pcd')
    print(pc_pcd)
    print('shape of points', np.asarray(pc_pcd.points).shape)
    print('Shape of colors', np.asarray(pc_pcd.colors).shape)
    #R = pc_pcd.get_rotation_matrix_from_xyz((0, 0,np.pi / 2))
    #pc_pcd.rotate(R, center=(0, 0, 0))
    #pc_pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
    o3d.visualization.draw_geometries([pc_pcd])

def npy_pc_visulization():
    '''
    This function uses open3d for visualizing pointcloud saved in a .npy format in python
    '''
    pc_np=np.load('./test.npy')
    print('Shape of point cloud: ',pc_np.shape)
    pc=o3d.geometry.PointCloud()
    pc.points=o3d.utility.Vector3dVector(pc_np)
    #o3d.visualization.draw_geometries([pc])

def demo_manual_registration():
    '''
    This function uses open3d for visualizing pointcloud in python
    '''
    
    print("Demo for manual ICP")
    #pcd_data = o3d.data.DemoICPPointClouds()
    source = o3d.io.read_point_cloud('./pointclouds/1666091265.036527329.pcd')
    #target = o3d.io.read_point_cloud(pcd_data.paths[2])
    #print("Visualization of two point clouds before manual alignment")
    #draw_registration_result(source, target, np.identity(4))

    # pick points from two point clouds and builds correspondences
    picked_id_source = pick_points(source)
    #picked_id_target = pick_points(target)
    # assert (len(picked_id_source) >= 3)
    # corr = np.zeros((len(picked_id_source), 2))
    # corr[:, 0] = picked_id_source
    # corr[:, 1] = picked_id_target

    # # estimate rough transformation using correspondences
    # print("Compute a rough transform using the correspondences given by user")
    # p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    # trans_init = p2p.compute_transformation(source, target,
    #                                         o3d.utility.Vector2iVector(corr))

    # # point-to-point ICP for refinement
    # print("Perform point-to-point ICP refinement")
    # threshold = 0.03  # 3cm distance threshold
    # reg_p2p = o3d.pipelines.registration.registration_icp(
    #     source, target, threshold, trans_init,
    #     o3d.pipelines.registration.TransformationEstimationPointToPoint())
    # draw_registration_result(source, target, reg_p2p.transformation)
    # print("")

def pick_points(pcd):
    '''
    This function uses open3d for visualizing pointcloud in python and picking specifc points from it
    '''
    
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()

def draw_registration_result(source, target, transformation):
    '''
    This function uses open3d for visualizing pointcloud in python
    '''
    
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


bag=rosbag.Bag('rosbag_file.bag')

ground_bag=rosbag.Bag('reference_rosbag_file.bag')

#write_rosbag(bag)
#write_rosbag_ground(ground_bag)
#extract_lidar_point(bag)
pc_visualization()
#npy_pc_visulization()
#demo_manual_registration()
#pc_to_csv(bag)
#pointcloud_to_pcd(bag)
#print(pc_points)
