#pragma once
// C++
#include <deque>
#include <vector>
// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
// time
#include "tic_toc.h"
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>

class ExtractNode
{
public:
    ExtractNode();

private:
    // ros
    ros::NodeHandle nh_;
    // sub
    ros::Subscriber sub_livox_;                                                       // livox信息接收
    void LivoxPointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg); //点云回调
    // pub
    ros::Publisher pub_dump_;                       //特征点信息发布
    sensor_msgs::PointCloud2 dump_pointcloud2_;     // ros消息类型：pointcloud
    ros::Publisher pub_pretreat_;                   //特征点信息发布
    sensor_msgs::PointCloud2 pretreat_pointcloud2_; // ros消息类型：pointcloud

    // PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pointcloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pretreat_pointcloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr dump_pointcloud_;

    //点被筛选过标记
    int cloudPicked_[24000] = {0};
    int sample_count_;
    float max_x, min_x, max_y, min_y, max_z, min_z;
    std::vector<pcl::PointXYZ> slice_points_;

    //标志位
    bool is_find_A_;

    //系统初始化
    bool SystemInit();
    //消息类型转换
    // void ConvertPointCloud2PCL(const sensor_msgs::PointCloud::ConstPtr &cloud_msg);
    //点云聚类
    bool PointClustering();
    //位姿解算
    bool PoseResolving();
    //拟合求体积
    void VolumeMatching();
    //消息发布
    void MsgPub();
    //自定义排序
    static bool comp(pcl::PointXYZ &a, pcl::PointXYZ &b)
    {
        return (a.y < b.y);
    }
};
