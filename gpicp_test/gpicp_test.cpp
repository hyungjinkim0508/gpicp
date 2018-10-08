
#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>

#include "src/gpicp/gpicp.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpicp_test");
  ros::NodeHandle n;
  ros::Publisher pub_cloudQuery = n.advertise<sensor_msgs::PointCloud2>("cloudQuery", 1);
  ros::Publisher pub_cloudTarget = n.advertise<sensor_msgs::PointCloud2>("cloudTarget", 1);
  ros::Publisher pub_cloudResultGPICP = n.advertise<sensor_msgs::PointCloud2>("cloudResultGPICP", 1);
  ros::Publisher pub_cloudResultGICP = n.advertise<sensor_msgs::PointCloud2>("cloudResultGICP", 1);

  // Data load process
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudQuery(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarget(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile("velodyneCloud_1.pcd", *cloudQuery);
  pcl::io::loadPCDFile("velodyneCloud_2.pcd", *cloudTarget);


  // GP-ICP preocess
  std::cout << "GP-ICP start!" << std::endl;
  pcl::PointCloud<pcl::PointXYZ> cloudResult_GPICP;
  pcl::GeneralizedIterativeClosestPoint_GP<pcl::PointXYZ, pcl::PointXYZ> gpicp;
  gpicp.setInputSource(cloudQuery);
  gpicp.setInputTarget(cloudTarget);  
  gpicp.align(cloudResult_GPICP);
  Eigen::Matrix4f GPICP_result = gpicp.getFinalTransformation();
  pcl::transformPointCloud(*cloudQuery,cloudResult_GPICP,GPICP_result);

  std::cout << "GP-ICP Result transformation" << std::endl;
  std::cout << GPICP_result << std::endl;

  // G-ICP process
  std::cout << "G-ICP start!" << std::endl;
  pcl::PointCloud<pcl::PointXYZ> cloudResult_GICP;
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setInputSource(cloudQuery);
  gicp.setInputTarget(cloudTarget);
  gicp.align(cloudResult_GICP);
  Eigen::Matrix4f GICP_result = gicp.getFinalTransformation();
  pcl::transformPointCloud(*cloudQuery,cloudResult_GICP,GICP_result);

  std::cout << "G-ICP Result transformation" << std::endl;
  std::cout << GICP_result << std::endl;

  // Display process in rviz
  sensor_msgs::PointCloud2 cloud_ROS_target;
  pcl::toROSMsg(*cloudQuery, cloud_ROS_target);
  cloud_ROS_target.header.frame_id = "map";


  sensor_msgs::PointCloud2 cloud_ROS_query;
  pcl::toROSMsg(*cloudTarget, cloud_ROS_query);
  cloud_ROS_query.header.frame_id = "map";


  sensor_msgs::PointCloud2 cloud_ROS_GPICPResult;
  pcl::toROSMsg(cloudResult_GPICP, cloud_ROS_GPICPResult);
  cloud_ROS_GPICPResult.header.frame_id = "map";


  sensor_msgs::PointCloud2 cloud_ROS_GICPResult;
  pcl::toROSMsg(cloudResult_GICP, cloud_ROS_GICPResult);
  cloud_ROS_GICPResult.header.frame_id = "map";


  std::cout << "Process done" << std::endl;
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    pub_cloudQuery.publish(cloud_ROS_target);
    pub_cloudTarget.publish(cloud_ROS_query);
    pub_cloudResultGPICP.publish(cloud_ROS_GPICPResult);
    pub_cloudResultGICP.publish(cloud_ROS_GICPResult);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
