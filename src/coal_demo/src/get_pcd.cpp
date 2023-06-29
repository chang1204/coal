#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

void msg_callbak(const sensor_msgs::PointCloud2::ConstPtr &pcl2Msg){
  
  std::string pkg_path = ros::package::getPath("coal_demo");
  YAML::Node config = YAML::LoadFile(pkg_path+"/yaml/coal.yaml");
  std::string file_path = config["get_pcd"]["file_path"].as<std::string>();

  // Save PCD file
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*pcl2Msg,*cloud);
  pcl::io::savePCDFileASCII (file_path, *cloud);
  std::cerr << "Saved " << cloud->points.size () << " data points to "<< file_path << std::endl;
  
}
int main (int argc, char *argv[])
{
  ros::init(argc,argv,"get_pcd");
  ros::NodeHandle nh;
  ros::Subscriber subpcl2 = nh.subscribe<sensor_msgs::PointCloud2>("/pcl_2_pcl2",10,&msg_callbak);

  ros::spin();
  return 0;
}
