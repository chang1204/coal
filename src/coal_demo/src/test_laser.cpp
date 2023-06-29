//使用ros接受雷达点云信息
#include "coal.h"

pcl::PointCloud<pcl::PointXYZI>::Ptr laserMsgInfo;
pcl::PointCloud<pcl::PointXYZI>::Ptr laserMsgInfo2;

void laserCallback(const sensor_msgs::PointCloud2::ConstPtr &laserMsg){

    laserMsgInfo.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserMsgInfo->clear();
    pcl::fromROSMsg(*laserMsg,*laserMsgInfo);
    std::cout<< "雷达强度信息:"<<laserMsgInfo->points[0].intensity<<std::endl;

}

void laserCallback2(const sensor_msgs::PointCloudConstPtr &laserMsg){

    laserMsgInfo2.reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserMsgInfo2->clear();

    sensor_msgs::PointCloud2 pcl2;
    sensor_msgs::convertPointCloudToPointCloud2(*laserMsg , pcl2);
    pcl::fromROSMsg(pcl2,*laserMsgInfo2);
    std::cout<< "雷达强度信息:"<<laserMsgInfo2->points[0].intensity<<std::endl;

}

int main(int argc , char *argv[]){
    ros::init(argc,argv,"test_laser");
    ros::NodeHandle nh;

    // ros::Subscriber subLaser;
    // subLaser = nh.subscribe<sensor_msgs::PointCloud2>("/pcl_2_pcl2",100,&laserCallback);

    ros::Subscriber subLaser2;
    subLaser2 = nh.subscribe<sensor_msgs::PointCloud>("/scan" , 100 , &laserCallback2);

    ros::spin();

    return 0;
}