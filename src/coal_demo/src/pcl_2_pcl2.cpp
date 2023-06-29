#include "coal.h"

class Convert
{
private:
    ros::NodeHandle nh;
    ros::Subscriber subLaserScan;
    ros::Publisher m_pub_pointcloud2;
    pcl::PointCloud<pcl::PointXYZI> laserMsgInfo;
    pcl::PointCloud<pcl::PointXYZI> puLaserMsg;

    int count = 0;

public:
    Convert():
    nh("~")
    {    
        // \033[1;32m，\033[0m 终端显示成绿色

        ROS_INFO_STREAM("\033[1;32m----> PLICP odometry started.\033[0m");
        // 接收
        subLaserScan=nh.subscribe<sensor_msgs::PointCloud> ("/scan",1,&Convert::laserScanCall,this);

        // 发布
        m_pub_pointcloud2 = nh.advertise<sensor_msgs::PointCloud2>( "/pcl_2_pcl2", 100,this );

    }

    void laserScanCall(const sensor_msgs::PointCloudConstPtr &laserMsg)
    {   
        sensor_msgs::PointCloud2 pcl2;
        sensor_msgs::convertPointCloudToPointCloud2(*laserMsg,pcl2);
        pcl::fromROSMsg(pcl2,laserMsgInfo);
        
        int num = laserMsgInfo.size();

        for(int i=0 ; i<num;i++){
            std::float_t range = (laserMsgInfo[i].x)*(laserMsgInfo[i].x)+(laserMsgInfo[i].y)*(laserMsgInfo[i].y)+(laserMsgInfo[i].z)*(laserMsgInfo[i].z);
            if( range < 900 && range > 0.01 ){
                // laserMsgInfo[i].intensity = 0;
                puLaserMsg.push_back(laserMsgInfo[i]);
            }
        }

        // 稠密点云
        // count ++;
        // if(count > 10){
        //     publishPoint( puLaserMsg );
        //     puLaserMsg.clear();    
        //     count = 0;        
        // }
        
        // 稀疏点云
        publishPoint( puLaserMsg );
        puLaserMsg.clear();  
    }

    void publishPoint(const pcl::PointCloud<pcl::PointXYZI> &laserInfo)
    {        
        sensor_msgs::PointCloud2::Ptr temPointCloud;
        temPointCloud.reset(new sensor_msgs::PointCloud2() );
        ros::Time currentTime;
        currentTime = ros::Time::now();

        pcl::toROSMsg(laserInfo,*temPointCloud);
        temPointCloud->header.stamp=currentTime;
        temPointCloud->header.frame_id="base_link";              
        m_pub_pointcloud2.publish(temPointCloud);          
    }
};
int main(int argc,char *argv[])
{   
    ros::init(argc,argv,"pcl_2_pcl2");
    Convert convert ;
    ros::spin();
    return 0;
}