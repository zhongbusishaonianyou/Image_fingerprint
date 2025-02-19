#include <ros/ros.h>
#include "binary_fingerprint.h"
#include "Preprocess.h"
using namespace std;

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "Binary_fingerprint");
    ros::NodeHandle nh;  
     
    Binary_Image_Fingerprint  BIF;
    Preprocess preprocess_data;
    
    size_t frame_id = 1;
    vector<int>::iterator it;
    std::ofstream BIF_loop(preprocess_data.BIF_loop_path, std::ios::app);
    BIF_loop.setf(std::ios::fixed, std::ios::floatfield);
    BIF_loop.precision(5);

    auto GT_loop= preprocess_data.get_GT_loop_closure(preprocess_data.GTposes_folder); 
    ros::Rate LiDAR_rate(100); //test has more speed
    while (ros::ok())
   {    
          bool loop_flag=0;
          BIF.makeAndSaveinfo(*(preprocess_data.get_downsample_data(frame_id)));
          
          if (frame_id>(size_t)preprocess_data.num_candidates)
           { 
       
            auto detectResult= BIF.detectLoopClosureID() ; 

           it=std::find(GT_loop[frame_id].begin(),GT_loop[frame_id].end(),detectResult.first);
           if(it!=GT_loop[frame_id].end()) loop_flag=1;
           BIF_loop << frame_id << " " << detectResult.first << " "<< detectResult.second<< " "<< loop_flag <<endl;
           } 
        frame_id++;
        ros::spinOnce();
        LiDAR_rate.sleep();
    }
    BIF_loop.close();
    return 0;
}

