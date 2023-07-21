import rosbag
import csv
import numpy as np
import open3d as o3d
from sensor_msgs import point_cloud2

def csm_processing():
    points_csm=np.loadtxt('cropped using point number_new/start_1666093055 - Cloud.csv',delimiter=';',skiprows=1)
    points_csm_x=points_csm[~np.all(points_csm==0, axis=1)]
    
    pcd_pc=o3d.geometry.PointCloud()
    pcd_pc.points=o3d.utility.Vector3dVector(points_csm_x)
    print('shape of points', np.asarray(pcd_pc.points).shape)

    '''Voxel Downsampling'''
    voxel_down_pcd = pcd_pc.voxel_down_sample(voxel_size=0.02)
    print('shape of voxel down pcd points', np.asarray(voxel_down_pcd.points).shape)

    #'''Uniform Downsampling'''
    #uni_down_pcd = pcd_pc.uniform_down_sample(every_k_points=5)
    #print('Shape of uniform down sample',np.asarray(uni_down_pcd.points).shape)
    #o3d.visualization.draw_geometries([uni_down_pcd])

    #'''Statistical outlier Removal'''
    #print('statistical outlier removal')
    #cl,ind=voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20,std_ratio=float(1.0))
    ##outlier_removed_pc=voxel_down_pcd.select_by_index(ind)
    ##display_inlier_outlier(voxel_down_pcd,ind)

    '''Radius outlier Removal'''
    print('Radius Outlier Removal')
    cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=16, radius=0.13)

    '''displaying inlier and outlier'''
    inlier_cloud = voxel_down_pcd.select_by_index(ind)
    #print('shape of inlier pointcloud ',np.asarray(inlier_cloud.points).shape)
    outlier_cloud = voxel_down_pcd.select_by_index(ind, invert=True)
    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    points_processed=np.asarray(inlier_cloud.points)
    print('Points of inlier pointcloud',points_processed.shape)
    x_values_crop=points_processed[:,0]
    x_values_crop=x_values_crop*np.cos(np.pi/4)
    
    return x_values_crop


def dtm_processing():
    points_csm=np.loadtxt('cropped using point number/ground__1670316850 - Cloud.csv',delimiter=';',skiprows=1)
    points_csm_x=points_csm[~np.all(points_csm==0, axis=1)]
    
    pcd_pc=o3d.geometry.PointCloud()
    pcd_pc.points=o3d.utility.Vector3dVector(points_csm_x)
    print('shape of points', np.asarray(pcd_pc.points).shape)

    '''Voxel Downsampling'''
    voxel_down_pcd = pcd_pc.voxel_down_sample(voxel_size=0.02)
    print('shape of voxel down pcd points', np.asarray(voxel_down_pcd.points).shape)

    #'''Uniform Downsampling'''
    #uni_down_pcd = pcd_pc.uniform_down_sample(every_k_points=5)
    #print('Shape of uniform down sample',np.asarray(uni_down_pcd.points).shape)
    #o3d.visualization.draw_geometries([uni_down_pcd])
#
    #'''Statistical outlier Removal'''
    #print('statistical outlier removal')
    #cl,ind=voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20,std_ratio=float(1.0))
    #outlier_removed_pc=voxel_down_pcd.select_by_index(ind)
    #display_inlier_outlier(voxel_down_pcd,ind)

    '''Radius outlier Removal'''
    print('Radius Outlier Removal')
    cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=16, radius=0.13)

    '''displaying inlier and outlier'''
    inlier_cloud = voxel_down_pcd.select_by_index(ind)
    #print('shape of inlier pointcloud ',np.asarray(inlier_cloud.points).shape)
    outlier_cloud = voxel_down_pcd.select_by_index(ind, invert=True)
    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    points_processed=np.asarray(inlier_cloud.points)
    print('Points of inlier pointcloud',points_processed.shape)
    x_values_dtm=points_processed[:,0]
    x_values_dtm=x_values_dtm*np.cos(np.pi/4)
    
    return x_values_dtm    

def Average(lst):
    '''
    '''
    
    return sum(lst) / len(lst)

crop_readings=csm_processing()
dtm_readings=dtm_processing()

crop_readings_average=Average(crop_readings)
dtm_readings_average=Average(dtm_readings)


aver=crop_readings_average-dtm_readings_average
print('Average = {}'.format(aver) )
maxim=max(crop_readings)-max(dtm_readings)
print('Max Value = {}'.format(maxim))
mini=min(crop_readings)-min(dtm_readings)
print('Min Value =  {}'.format(mini) )
max_min=max(crop_readings)-min(dtm_readings)
print('Max - Min Value = {}'.format(max_min))



























#def display_inlier_outlier(cloud, ind):
#    inlier_cloud = cloud.select_by_index(ind)
#    print('shape of inlier pointcloud ',np.asarray(inlier_cloud.points).shape)
#    outlier_cloud = cloud.select_by_index(ind, invert=True)
#
#    print("Showing outliers (red) and inliers (gray): ")
#    outlier_cloud.paint_uniform_color([1, 0, 0])
#    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
#    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

#points_csm=np.loadtxt('cropped using point number_new/start_1666093055 - Cloud.csv',delimiter=';',skiprows=1)
##print(points[0:2])
##print(points.shape())
#points_csm_x=points_csm[~np.all(points_csm==0, axis=1)]
#
#'''Creating and visulaizing original pointcloud'''
#pcd_pc=o3d.geometry.PointCloud()
#pcd_pc.points=o3d.utility.Vector3dVector(points_csm_x)
##o3d.io.write_point_cloud("./sync.pcd", pcd)
##o3d.visualization.draw_geometries([pcd_pc])
#print('shape of points', np.asarray(pcd_pc.points).shape)
#
#'''Voxel Downsampling'''
#voxel_down_pcd = pcd_pc.voxel_down_sample(voxel_size=0.02)
#print('shape of voxel down pcd points', np.asarray(voxel_down_pcd.points).shape)
##o3d.visualization.draw_geometries([voxel_down_pcd])
#
#'''Uniform Downsampling'''
##print("Every 5th points are selected")
##uni_down_pcd = pcd_pc.uniform_down_sample(every_k_points=5)
##print('Shape of uniform down sample',np.asarray(uni_down_pcd.points).shape)
##o3d.visualization.draw_geometries([uni_down_pcd])
#
#'''Statistical outlier Removal'''
##print('statistical outlier removal')
##cl,ind=voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20,std_ratio=float(1.0))
##outlier_removed_pc=voxel_down_pcd.select_by_index(ind)
##display_inlier_outlier(voxel_down_pcd,ind)
#
#'''Radius outlier Removal'''
#print('Radius Outlier Removal')
#cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=16, radius=0.13)
##display_inlier_outlier(voxel_down_pcd,ind)
#
#'''displaying inlier and outlier'''
#inlier_cloud = voxel_down_pcd.select_by_index(ind)
##print('shape of inlier pointcloud ',np.asarray(inlier_cloud.points).shape)
#outlier_cloud = voxel_down_pcd.select_by_index(ind, invert=True)
#
##print("Showing outliers (red) and inliers (gray): ")
##outlier_cloud.paint_uniform_color([1, 0, 0])
##inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
##o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
#
#points_processed=np.asarray(inlier_cloud.points)
#print('Points of inlier pointcloud',points_processed.shape)
##print(points_processed)
#print(points_processed[0:4,0])
#x_values_crop=points_processed[:,0]
#x_values_crop=x_values_crop*np.cos(np.pi/4)
##x_values_crop=(float(x_values_crop)*(np.cos(np.pi/4)))
#print(x_values_crop[0:4])