# import open3d as o3d

# if __name__ == "__main__":
#     sample_ply_data = o3d.data.PLYPointCloud()
#     pcd = o3d.io.read_point_cloud(sample_ply_data.path)
#     # Flip it, otherwise the pointcloud will be upside down.
#     pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
#     print(pcd)
#     axis_aligned_bounding_box = pcd.get_axis_aligned_bounding_box()
#     axis_aligned_bounding_box.color = (1, 0, 0)
#     oriented_bounding_box = pcd.get_oriented_bounding_box()
#     oriented_bounding_box.color = (0, 1, 0)
#     print(
#         "Displaying axis_aligned_bounding_box in red and oriented bounding box in green ..."
#     )
#     o3d.visualization.draw(
#         [pcd, axis_aligned_bounding_box, oriented_bounding_box])

##################################### point_cloud_crop.py ################################################

# import open3d as o3d

# if __name__ == "__main__":
#     print("Load a ply point cloud, crop it, and render it")
#     sample_ply_data = o3d.data.DemoCropPointCloud()
#     pcd = o3d.io.read_point_cloud(sample_ply_data.point_cloud_path)
#     vol = o3d.visualization.read_selection_polygon_volume(
#         sample_ply_data.cropped_json_path)
#     chair = vol.crop_point_cloud(pcd)
#     # Flip the pointclouds, otherwise they will be upside down.
#     pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
#     chair.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

#     print("Displaying original pointcloud ...")
#     o3d.visualization.draw([pcd])
#     print("Displaying cropped pointcloud")
#     o3d.visualization.draw([chair])

######################## Remove_geometry.py ################################

# import open3d as o3d
# import numpy as np
# import time
# import copy


# def visualize_non_blocking(vis, pcds):
#     for pcd in pcds:
#         vis.update_geometry(pcd)
#     vis.poll_events()
#     vis.update_renderer()


# pcd_data = o3d.data.PCDPointCloud()
# pcd_orig = o3d.io.read_point_cloud(pcd_data.path)
# flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
# pcd_orig.transform(flip_transform)
# n_pcd = 5
# pcds = []
# for i in range(n_pcd):
#     pcds.append(copy.deepcopy(pcd_orig))
#     trans = np.identity(4)
#     trans[:3, 3] = [3 * i, 0, 0]
#     pcds[i].transform(trans)

# vis = o3d.visualization.Visualizer()
# vis.create_window()
# start_time = time.time()
# added = [False] * n_pcd

# curr_sec = int(time.time() - start_time)
# prev_sec = curr_sec - 1

# while True:
#     curr_sec = int(time.time() - start_time)
#     if curr_sec - prev_sec == 1:
#         prev_sec = curr_sec

#         for i in range(n_pcd):
#             if curr_sec % (n_pcd * 2) == i and not added[i]:
#                 vis.add_geometry(pcds[i])
#                 added[i] = True
#                 print("Adding %d" % i)
#             if curr_sec % (n_pcd * 2) == (i + n_pcd) and added[i]:
#                 vis.remove_geometry(pcds[i])
#                 added[i] = False
#                 print("Removing %d" % i)

#     visualize_non_blocking(vis, pcds)



################################# Interactive Visualisation #######################################

# examples/python/visualization/interactive_visualization.py

# import numpy as np
# import copy
# import open3d as o3d


# def demo_crop_geometry():
#     print("Demo for manual geometry cropping")
#     print(
#         "1) Press 'Y' twice to align geometry with negative direction of y-axis"
#     )
#     print("2) Press 'K' to lock screen and to switch to selection mode")
#     print("3) Drag for rectangle selection,")
#     print("   or use ctrl + left click for polygon selection")
#     print("4) Press 'C' to get a selected geometry")
#     print("5) Press 'S' to save the selected geometry")
#     print("6) Press 'F' to switch to freeview mode")
#     pcd_data = o3d.data.DemoICPPointClouds()
#     pcd = o3d.io.read_point_cloud(pcd_data.paths[0])
#     o3d.visualization.draw_geometries_with_editing([pcd])


# def draw_registration_result(source, target, transformation):
#     source_temp = copy.deepcopy(source)
#     target_temp = copy.deepcopy(target)
#     source_temp.paint_uniform_color([1, 0.706, 0])
#     target_temp.paint_uniform_color([0, 0.651, 0.929])
#     source_temp.transform(transformation)
#     o3d.visualization.draw_geometries([source_temp, target_temp])


# def pick_points(pcd):
#     print("")
#     print(
#         "1) Please pick at least three correspondences using [shift + left click]"
#     )
#     print("   Press [shift + right click] to undo point picking")
#     print("2) After picking points, press 'Q' to close the window")
#     vis = o3d.visualization.VisualizerWithEditing()
#     vis.create_window()
#     vis.add_geometry(pcd)
#     vis.run()  # user picks points
#     vis.destroy_window()
#     print("")
#     return vis.get_picked_points()


# def demo_manual_registration():
#     print("Demo for manual ICP")
#     pcd_data = o3d.data.DemoICPPointClouds()
#     source = o3d.io.read_point_cloud(pcd_data.paths[0])
#     target = o3d.io.read_point_cloud(pcd_data.paths[2])
#     print("Visualization of two point clouds before manual alignment")
#     draw_registration_result(source, target, np.identity(4))

#     # pick points from two point clouds and builds correspondences
#     picked_id_source = pick_points(source)
#     picked_id_target = pick_points(target)
#     assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
#     assert (len(picked_id_source) == len(picked_id_target))
#     corr = np.zeros((len(picked_id_source), 2))
#     corr[:, 0] = picked_id_source
#     corr[:, 1] = picked_id_target

#     # estimate rough transformation using correspondences
#     print("Compute a rough transform using the correspondences given by user")
#     p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
#     trans_init = p2p.compute_transformation(source, target,
#                                             o3d.utility.Vector2iVector(corr))

#     # point-to-point ICP for refinement
#     print("Perform point-to-point ICP refinement")
#     threshold = 0.03  # 3cm distance threshold
#     reg_p2p = o3d.pipelines.registration.registration_icp(
#         source, target, threshold, trans_init,
#         o3d.pipelines.registration.TransformationEstimationPointToPoint())
#     draw_registration_result(source, target, reg_p2p.transformation)
#     print("")


# if __name__ == "__main__":
#     #demo_crop_geometry()
#     demo_manual_registration()


##########################Checking Rotation ##################################################################


import numpy as np
import copy
import open3d as o3d

# pointcloud=o3d.io.read_point_cloud('./greenland.pcd')
# pointcloud_r=copy.deepcopy(pointcloud)

# R = pointcloud.get_rotation_matrix_from_xyz((0,np.pi/4, 0))
# pointcloud_r.rotate(R, center=(0, 0, 0))





# o3d.io.write_point_cloud("greenland_rotated_ccw.pcd", pointcloud_r)
# ##o3d.io.write_point_cloud("greenland_rotated_cw.csv", pointcloud_r)
# print(np.asarray(pointcloud_r.points))
# xyz_load=np.asarray(pointcloud_r.points)
# print('Shape of original pcd {}'.format(np.shape(pointcloud.points)))
# print('Shape of rotated pcd {}'.format(np.shape(pointcloud_r.points)))
# with open('./greenland_rotated_ccw.csv', 'w') as f:
#     for point in xyz_load:
#         f.write(','.join(str(x) for x in point) + '\n')

def data_with_beamangle(pcd_points):
    
    beam_angle=0.519*np.pi/180
    lidar_angle=np.pi/4

    pcd_points[0:96,-1]=pcd_points[0:96,-1]*np.cos(31*beam_angle)
    pcd_points[96:193,-1]=pcd_points[96:193,-1]*np.cos(30*beam_angle)
    pcd_points[193:291,-1]=pcd_points[193:291,-1]*np.cos(29*beam_angle)
    pcd_points[291:391,-1]=pcd_points[291:391,-1]*np.cos(28*beam_angle)
    pcd_points[391:492,-1]=pcd_points[391:492,-1]*np.cos(27*beam_angle)
    pcd_points[492:594,-1]=pcd_points[492:594,-1]*np.cos(26*beam_angle)
    pcd_points[594:696,-1]=pcd_points[594:696,-1]*np.cos(25*beam_angle)
    pcd_points[696:799,-1]=pcd_points[696:799,-1]*np.cos(24*beam_angle)
    pcd_points[799:904,-1]=pcd_points[799:904,-1]*np.cos(23*beam_angle)
    pcd_points[904:1010,-1]=pcd_points[904:1010,-1]*np.cos(22*beam_angle)
    pcd_points[1010:1116,-1]=pcd_points[1010:1116,-1]*np.cos(21*beam_angle)
    pcd_points[1116:1224,-1]=pcd_points[1116:1224,-1]*np.cos(20*beam_angle)
    pcd_points[1224:1333,-1]=pcd_points[1224:1333,-1]*np.cos(19*beam_angle)
    pcd_points[1333:1444,-1]=pcd_points[1333:1444,-1]*np.cos(18*beam_angle)
    pcd_points[1444:1555,-1]=pcd_points[1444:1555,-1]*np.cos(17*beam_angle)
    pcd_points[1555:1668,-1]=pcd_points[1555:1668,-1]*np.cos(16*beam_angle)
    pcd_points[1668:1782,-1]=pcd_points[1668:1782,-1]*np.cos(15*beam_angle)
    pcd_points[1782:1896,-1]=pcd_points[1782:1896,-1]*np.cos(14*beam_angle)
    pcd_points[1896:2012,-1]=pcd_points[1896:2012,-1]*np.cos(13*beam_angle)
    pcd_points[2012:2128,-1]=pcd_points[2012:2128,-1]*np.cos(12*beam_angle)
    pcd_points[2128:2246,-1]=pcd_points[2128:2246,-1]*np.cos(11*beam_angle)
    pcd_points[2246:2365,-1]=pcd_points[2246:2365,-1]*np.cos(10*beam_angle)
    pcd_points[2365:2485,-1]=pcd_points[2365:2485,-1]*np.cos(9*beam_angle)
    pcd_points[2485:2605,-1]=pcd_points[2485:2605,-1]*np.cos(8*beam_angle)
    pcd_points[2605:2727,-1]=pcd_points[2605:2727,-1]*np.cos(7*beam_angle)
    pcd_points[2727:2849,-1]=pcd_points[2727:2849,-1]*np.cos(6*beam_angle)
    pcd_points[2849:2973,-1]=pcd_points[2849:2973,-1]*np.cos(5*beam_angle)
    pcd_points[2973:3099,-1]=pcd_points[2973:3099,-1]*np.cos(4*beam_angle)
    pcd_points[3099:3225,-1]=pcd_points[3099:3225,-1]*np.cos(3*beam_angle)
    pcd_points[3225:3353,-1]=pcd_points[3225:3353,-1]*np.cos(2*beam_angle)
    pcd_points[3353:3482,-1]=pcd_points[3353:3482,-1]*np.cos(beam_angle)
    pcd_points[3482:3611,-1]=pcd_points[3482:3611,-1]
    pcd_points[3611:3741,-1]=pcd_points[3611:3741,-1]*np.cos(-beam_angle)
    pcd_points[3741:3872,-1]=pcd_points[3741:3872,-1]*np.cos(-2*beam_angle)
    pcd_points[3872:4005,-1]=pcd_points[3872:4005,-1]*np.cos(-3*beam_angle)
    pcd_points[4005:4139,-1]=pcd_points[4005:4139,-1]*np.cos(-4*beam_angle)
    pcd_points[4139:4274,-1]=pcd_points[4139:4274,-1]*np.cos(-5*beam_angle)
    pcd_points[4274:4410,-1]=pcd_points[4274:4410,-1]*np.cos(-6*beam_angle)
    pcd_points[4410:4546,-1]=pcd_points[4410:4546,-1]*np.cos(-7*beam_angle)
    pcd_points[4546:4684,-1]=pcd_points[4546:4684,-1]*np.cos(-8*beam_angle)
    pcd_points[4684:4822,-1]=pcd_points[4684:4822,-1]*np.cos(-9*beam_angle)
    pcd_points[4822:4961,-1]=pcd_points[4822:4961,-1]*np.cos(-10*beam_angle)
    pcd_points[4962:5103,-1]=pcd_points[4962:5103,-1]*np.cos(-11*beam_angle)
    pcd_points[5103:5245,-1]=pcd_points[5103:5245,-1]*np.cos(-12*beam_angle)
    pcd_points[5245:5389,-1]=pcd_points[5245:5389,-1]*np.cos(-13*beam_angle)
    pcd_points[5389:5534,-1]=pcd_points[5389:5534,-1]*np.cos(-14*beam_angle)
    
    #pcd_points[:,-1]=np.abs(pcd_points[:,-1])
    return pcd_points


x_min=2.9
x_max=3.9
y_min=-0.5
y_max=0.5
z_min=-10
z_max=10

point_cloud=o3d.io.read_point_cloud('./greenland.pcd')
point_cloud_ccw=copy.deepcopy(point_cloud)
R=point_cloud.get_rotation_matrix_from_xyz((0,np.pi/4,0))
point_cloud_ccw.rotate(R, center=(0, 0, 0))


#pcd_ccw_final=data_with_beamangle(pcd_ccw_xyz)
#print(pcd_ccw_final.shape)
#pcd_final=o3d.geometry.PointCloud()
#pcd_final.points=o3d.utility.Vector3dVector(pcd_ccw_final)

bbox_full=o3d.geometry.AxisAlignedBoundingBox(
    min_bound=[2.3, -2.0, -10],
    max_bound=[6.59, 2.0, 10]
)

point_cloud_crop_front=point_cloud_ccw.crop(bbox_full)

pcd_front_xyz=np.asarray(point_cloud_crop_front.points)
pcd_front_with_angle_points=data_with_beamangle(pcd_front_xyz)
pcd_front_with_angle=o3d.geometry.PointCloud()
pcd_front_with_angle.points=o3d.utility.Vector3dVector(pcd_front_with_angle_points)

o3d.io.write_point_cloud('pcd_crop_front_with_angle.pcd', pcd_front_with_angle)
o3d.io.write_point_cloud('pcd_crop_front.pcd', point_cloud_crop_front)

bbox_roi = o3d.geometry.AxisAlignedBoundingBox(
    min_bound=[2.9, -0.5, -10],
    max_bound=[3.99, 0.5, 10]
)
pointcloud_roi_full = pcd_front_with_angle.crop(bbox_roi)
o3d.io.write_point_cloud('pcd_roi_with_angle.pcd', pointcloud_roi_full)

pcd_array=np.array([])

for x in np.arange(x_min,x_max,0.2):
    for y in np.arange(y_min,y_max,0.2):    
        bbox=o3d.geometry.AxisAlignedBoundingBox(
            min_bound=[x,y,z_min],
            max_bound=[x+0.2,y+0.2,z_max]
        )
        pcd=pointcloud_roi_full.crop(bbox)
        pcd_array=np.append(pcd_array,pcd)



for i in range(0,len(pcd_array)):
    o3d.io.write_point_cloud('pcd_{}.pcd'.format(i), pcd_array[i])


