### usage

#### 1. Requirements

1. cmake
2. PCL
3. OpenCV
4. python
5. Eigen3

#### 2. Run

cd ~/catkin_ws/src
download BIF-main file 
cd ..
catkin_make 
source devel/setup.bash
rosrun fingerprint binary_fingerprint 

#### 3.visualize pr curve

1. please open draw.py file and revise  sequence 

2. python draw.py

3. you will see  following images:

   |                                                     KITTI 00 |                                                              |
   | ------------------------------------------------------------ | ------------------------------------------------------------ |
   | ![00](https://github.com/user-attachments/assets/7e78231c-e5a4-4e4b-b23b-e07cbefb048d)  |![00_traj](https://github.com/user-attachments/assets/d2480c5c-3e57-4201-bf06-964fa08fdc93)


   

   |                                                     KITTI 02 |                                                              |
   | ------------------------------------------------------------ | ------------------------------------------------------------ |
   |    ![02](https://github.com/user-attachments/assets/8d627c37-eae3-4b07-99ee-681170524428) |![02_traj](https://github.com/user-attachments/assets/35d14a09-2412-4d67-9ad1-a4e5e7083e3b)


   |                                                     KITTI 08 |                                                              |
   | ------------------------------------------------------------ | ------------------------------------------------------------ |
   | ![08](https://github.com/user-attachments/assets/bc38d88a-ba8c-4a94-9b80-2f2e217e58ef)  |![08_traj](https://github.com/user-attachments/assets/4d81fc12-3c76-4226-9986-0aaf7de7a9ec)


   

#### 4. cite



```
@ARTICLE{10187686, 
    author={Zhang, Guangyi and Zhang, Tao and Zhao, Shenggen and Hou, Lanhua}, 
    journal={IEEE Robotics and Automation Letters}, 
    title={Binary Image Fingerprint: Stable Structure Identifier for 3D LiDAR Place Recognition}, 
    year={2023}, volume={8}, number={9}, pages={5648-5655},  
    doi={10.1109/LRA.2023.3297063}
    }
@article{cui2022bow3d,
  title={BoW3D: Bag of Words for Real-Time Loop Closing in 3D LiDAR SLAM},
  author={Cui, Yunge and Chen, Xieyuanli and Zhang, Yinlong and Dong, Jiahua and Wu, Qingxiao and Zhu, Feng},
  journal={IEEE Robotics and Automation Letters},
  year={2022},
  publisher={IEEE}
}
@INPROCEEDINGS{9341010,
  author={Wang, Ying and Sun, Zezhou and Xu, Cheng-Zhong and Sarma, Sanjay E. and Yang, Jian and Kong, Hui},
  booktitle={2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={LiDAR Iris for Loop-Closure Detection}, 
  year={2020},
  volume={},
  number={},
  pages={5769-5775},
  keywords={Iris;Three-dimensional displays;Laser radar;Protocols;Image representation;Robot sensing systems;Intelligent robots},
  doi={10.1109/IROS45743.2020.9341010}
  }

```
