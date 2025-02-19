#pragma once

#include <ros/ros.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include<pcl/kdtree/kdtree_flann.h>

#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <string>
using namespace std;
class Preprocess
{

public: 
     Preprocess()=default;
     //
     const float revisit_thres=4;
     //times=30s;exclude candidates;
     const int num_candidates=300;
     //downsample size (in paper)
     const float filter_size = 1.2;

     const string seq = "08";
     const string dataset_folder = "/media/zgy/zgy/loop_closure/"+seq +"/velodyne/";
     const string GTposes_folder="/home/zgy/ROS/Binary_fingerprint/src/BIF-main/poses/"+seq +".txt";
     const string GTloop_save_path="/home/zgy/ROS/Binary_fingerprint/src/BIF-main/gt_loop/gt_loop_"+ seq + ".txt";
     const string BIF_loop_path = "/home/zgy/ROS/Binary_fingerprint/src/BIF-main/results/BIF_loop_"+seq+".txt";
public:
     vector<float> read_lidar_data(const std::string lidar_data_path);
     pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(vector<float> & lidar_data);
     pcl::PointCloud<pcl::PointXYZI>::Ptr read_poses_data(const string& pose_path);
     std::vector<std::vector<int>> calculate_GT_loop(pcl::PointCloud<pcl::PointXYZI>::Ptr & GTposes);
     std::vector<std::vector<int>> get_GT_loop_closure(const string& pose_path);
     pcl::PointCloud<pcl::PointXYZ>::Ptr get_downsample_data(const size_t & ID);
         
};