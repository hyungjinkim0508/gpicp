## GP-ICP (Ground Plane-ICP)

#### Abstract
In this paper, we propose a robust point cloud registration method for ground vehicles. Given the vast developments in the field of autonomous vehicles, the use of point cloud data has increased. The simultaneous localization and mapping (SLAM) algorithm is typically used to generate sophisticated point cloud maps. In the SLAM algorithm, the quality of the map depends on the performance of loop closure algorithms. The iterative closest point (ICP) algorithm is widely used for loop closure of the point cloud. However, the ICP algorithm might not work well for ground vehicles because it was originally developed for 3D reconstruction in computer vision field. Therefore, this paper proposes a method to find a robust matching correspondences in the ICP algorithm on ground vehicle conditions. The performance of the proposed method is compared with other conventional methods by using KITTI open datasets.

#### Reliability
The code is tested successfully at
* Linux 16.04 LTS
* ROS Kinetic

#### Current Status

Date: 22/FEB/2019 \
Version : 0.0.2 \
Note: master branch

### Result

* Initial point cloud \
Cyan: target point cloud \
Red: initial point cloud \
![Image of initialpose](/initialresult.png)

* G-ICP result (comparison method)\
Cyan: target point cloud \
Yellow: G-ICP point cloud \
![Image of GICPpose](/G-ICPresult.png)

* GP-ICP result (**Propsed method**) \
Cyan: target point cloud \
Magenta: GP-ICP point cloud \
![Image of GPICPpose](/GP-ICPresult.png)



### How to run

Compile with 'catkin_make' \
rosrun gpicp gpicp_test (you should run at the same folder with files velodyneCloud_1.pcd, ~_2.pcd) \
Check the result with rviz (load 'rviz_conf.rivz)

#### License

The code is under BSD-License.

#### Contact
Any suggestions or improvements are welcome. Feel free to contact me at hjkim86@kaist.ac.kr. \
Urban Robotics Lab (http://urobot.kaist.ac.kr)
