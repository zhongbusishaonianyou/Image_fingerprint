#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"

#include "tictoc.h"

using namespace Eigen;
using namespace nanoflann;
using namespace std;

using std::cout;
using std::endl;
using std::make_pair;

using std::atan2;
using std::cos;
using std::sin;


using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;

std::vector<float> eig2stdvec( MatrixXd _eigmat );
typedef pcl::PointXYZ  PointType;

class Binary_Image_Fingerprint
{
public: 
    Binary_Image_Fingerprint( ) = default; // reserving data space (of std::vector) could be considered. but the descriptor is lightweight so don't care.
    cv::Mat1b make_Structural_feature_matrix(const pcl::PointCloud<PointType> &cloud);
    MatrixXd makeRingkeys(cv::Mat1b & sfm);
    cv::Mat1b makeViewrim( cv::Mat1b &sfm);
    cv::Mat1b makeBinaryfingerprint(cv::Mat1b & sfm);
    
    int Align_Viewrim( cv::Mat1b & _viewrim1, cv::Mat1b & _viewrim2); 
    double calculate_hamming_dis (cv::Mat1b &_Fingerprint1,cv::Mat1b &_Fingerprint2 ); 
    std::pair<double, int> similarity_score( cv::Mat1b &curr_bif1,cv::Mat1b & curr_viewrim,cv::Mat1b &desc2,cv::Mat1b &candi_bif1,cv::Mat1b & candi_viewrim ); 

    // User-side API
    void makeAndSaveinfo( pcl::PointCloud<PointType> & _scan_down );
    std::pair<int, float> detectLoopClosureID( void ); // int: nearest node index, float: relative yaw  

public:
    
    const int    num_rings = 80;    
    const int    num_sectors = 120; // This parameter can be set to 180 or 360, and the effect of BIF is not significantly different.
    const double Max_range = 80.0; // This parameter does not need to be changed.
    const int    FINGERPRINT_SIZE=16; //Experiments were conducted on different datasets, and this parameter can be considered as a fixed value

    const double PC_UNIT_SECTORANGLE = 360.0 / double(num_sectors);
    const double PC_UNIT_RINGGAP = Max_range / double(num_rings);
    
    // tree
    const int    NUM_EXCLUDE_RECENT = 300; // simply just keyframe gap, but node position distance-based exclusion is ok. 
    const int    NUM_CANDIDATES_FROM_TREE = 20; // 10~20 is enough. 

    // loop thres
    const double HAMMING_DIST_THRES = 0.25; //Try not to exceed 0.25 as much as possible
    
    // config 
    const int    TREE_MAKING_PERIOD_ = 10; // i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost / in the LeGO-LOAM integration, it is synchronized with the loop detection callback (which is 1Hz) so it means the tree is updated evrey 10 sec. But you can use the smaller value because it is enough fast ~ 5-50ms wrt N.
    int          tree_making_period_counter = 0;

    // data 
    std::vector<cv::Mat1b> SFM;
    std::vector<Eigen::MatrixXd> Ring_keys;
    std::vector<cv::Mat1b> BIF;
    std::vector<cv::Mat1b> View_Rims;

    KeyMat Ringkeys_mat;
    KeyMat Ringkeys_to_search_;
    std::unique_ptr<InvKeyTree> Ringkeys_tree_;

}; 


