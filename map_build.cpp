//
// Created by zlp on 2021/4/21.
//
#include <dirent.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/auto_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
int main(){

  std::vector<Eigen::Matrix4d ,Eigen::aligned_allocator<Eigen::Matrix4d>> Poses;
  ifstream pose("traj_GICP_campus.txt");
  std::string line;
  while (getline(pose, line)) {
    if (line.size() > 0) {
      std::stringstream ss(line);
      Eigen::Matrix4d Pose_(Eigen::Matrix4d::Identity());
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j) {
          ss >> Pose_(i, j);

        }
      Poses.push_back(Pose_);
    }
  }
  pose.close();

  vector<string> lidar_files;
  string  path_lidar = "/media/sakura/800b229e-f13a-614f-93fc-bb3803996c63/Yhs_Robot/laser_210420/";
  struct dirent *ptr;
  DIR *dir;
  dir=opendir(path_lidar.c_str());
  while(ptr=readdir(dir)){
    if (ptr->d_name[0] == '.') continue;
    lidar_files.push_back(path_lidar+ptr->d_name);
  }
  sort(lidar_files.begin(),lidar_files.end());
  closedir(dir);

  assert(lidar_files.size()==Poses.size());

  double downsample_resolution = 0.4;
  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);

  pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);
  for (int i=0;i<lidar_files.size();i+=5){
    if (i%20==0){
      printf("\033[1A \033[1;31m  %d cloud_size: %d\n \033[0m", i,map->size());
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::load(lidar_files[i],*Cloud);
    voxelgrid.setInputCloud(Cloud);
    voxelgrid.filter(*Cloud);
    pcl::transformPointCloud(*Cloud,*Cloud,Poses[i]);
    *map+=*Cloud;
  }

  voxelgrid.setInputCloud(map);
  voxelgrid.filter(*map);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_cloud_color_handler(map, "intensity");
  viewer->addPointCloud< pcl::PointXYZI >(map, point_cloud_color_handler, "id");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "id");
  while (!viewer->wasStopped())
    viewer->spinOnce();

  pcl::io::save("campus.ply",*map);


  
}