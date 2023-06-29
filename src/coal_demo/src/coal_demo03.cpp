/*
 * @Author: changsaier changsaier@icloud.com
 * @Date: 2022-11-02 10:22:40
 * @LastEditors: changsaier changsaier@icloud.com
 * @LastEditTime: 2022-11-14 20:42:40
 * @FilePath: /coal/src/coal_demo/src/coal_demo02.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include"coal.h"


class ScanFeature
{
private:
    ros::NodeHandle nh;
    // ros::NodeHandle pnh;
    ros::Subscriber subLaserScan;
    ros::Publisher pubWithoutGround;
    ros::Publisher pubAllPoint;
    ros::Publisher pubClusterMsg;
    ros::Publisher pubcone;

    pcl::PointCloud<PointType>::Ptr laserMsgInfo;
    
    vector<pointInfo> pointInfoVec;
    pcl::PointCloud<PointType> pubPoint;
    pcl::PointCloud<PointType>::Ptr pointWithoutGround;
    pcl::PointCloud<PointType>::Ptr clusterPointCloud;
    pcl::PointCloud<PointType>::Ptr cloudFilter;
    pcl::PointCloud<PointType>::Ptr allPointCloud;

    pcl::console::TicToc tt;

    size_t cloudPointSize;
    bool systemInit ;
    int systemInitCount;
    float mountAngle=0;
    int scanNumb;

public:

    ScanFeature():
    nh("~")
    {
        // 初始化
        initParam();        
        // \033[1;32m，\033[0m 终端显示成绿色
        ROS_INFO_STREAM("\033[1;32m----> PLICP odometry started.\033[0m");
        // 接收
        // subLaserScan=nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar",100,&ScanFeature::laserScanCall,this);
        subLaserScan=nh.subscribe<sensor_msgs::PointCloud>("/scan",100,&ScanFeature::laserScanCall,this);

        // 发布
        pubWithoutGround=nh.advertise<sensor_msgs::PointCloud2>("withoutUnderground",100,this);
        pubAllPoint=nh.advertise<sensor_msgs::PointCloud2>("allPoint",100,this);
        pubClusterMsg=nh.advertise<sensor_msgs::PointCloud2>("clusterPC",100,this);

        pubcone=nh.advertise<sensor_msgs::PointCloud2>("pubcone",100,this);



    }
public:
    // void laserScanCall(const sensor_msgs::PointCloud2::ConstPtr &laserCloudMsg)
    // {
    //     if(!systemInit)
    //     {
    //         systemInitCount++;
    //         if(systemInitCount>10)
    //         {
    //             systemInit=true;
    //         }
    //         else
    //          return;
    //     }
        
    //     pcl::fromROSMsg(*laserCloudMsg,*laserMsgInfo);
    //     cloudPointSize=laserMsgInfo->points.size();
        
    //     std::cout<<cloudPointSize<<std::endl;
    //     //标记地面点云
    //     undergroundLabel();
    //     //点云分割

    //     pointCloudSegmentation();

    //     if(scanNumb<10)
    //         scanNumb++;
    //     else
    //     {
    //         pointCloudClustering(pointWithoutGround);
    //         // test(pointWithoutGround); 
    //         // calculateVolume();
    //         calculater();
    //         scanNumb=0;   
    //         pointWithoutGround->clear(); 
    //         allPointCloud->clear();    
    //     }

    //     //发布话题
    //     publishPoint();
    // }
    void laserScanCall(const sensor_msgs::PointCloudConstPtr &laserMsg)
    {
        if(!systemInit)
        {
            systemInitCount++;
            if(systemInitCount>10)
            {
                systemInit=true;
            }
            else
             return;
        }
        sensor_msgs::PointCloud2 PC;
        convertPointCloudToPointCloud2(*laserMsg,PC);
        pcl::fromROSMsg(PC,*laserMsgInfo);
        cloudPointSize=laserMsgInfo->points.size();
        
        std::cout<<cloudPointSize<<std::endl;
        //标记地面点云
        undergroundLabel();
        //点云分割

        pointCloudSegmentation();

        if(scanNumb<10)
            scanNumb++;
        else
        {
            pointCloudClustering(pointWithoutGround);
            // test(pointWithoutGround); 
            // calculateVolume();
            calculater();
            scanNumb=0;   
            pointWithoutGround->clear(); 
            allPointCloud->clear();    
        }

        //发布话题
        publishPoint();
    }
    void initParam()
    {
        systemInitCount = 0;
        systemInit = false;
        cloudPointSize=0;
        scanNumb=0;
        laserMsgInfo.reset(new pcl::PointCloud<PointType> ());
        laserMsgInfo->clear();
        pointWithoutGround.reset(new pcl::PointCloud<PointType>);
        clusterPointCloud.reset(new pcl::PointCloud<PointType>() );
        clusterPointCloud->clear();
        cloudFilter.reset(new pcl::PointCloud<PointType> () );
        allPointCloud.reset(new pcl::PointCloud<PointType> ());

    }
    // 对地面点云进行标记
    void undergroundLabel()
    {
        pointInfoVec.resize(cloudPointSize);
        pointInfoVec.clear();
        for(int i=0;i<cloudPointSize;i++)
        {
            float diffx,diffy,diffz,anglePitch,angleYaw;
            diffx=laserMsgInfo->points[i+1].x-laserMsgInfo->points[i].x;
            diffy=laserMsgInfo->points[i+1].y-laserMsgInfo->points[i].y;
            diffz=laserMsgInfo->points[i+1].z-laserMsgInfo->points[i].z;
            anglePitch=atan2(diffz,sqrt(diffx*diffx+diffy*diffy))*180/PI;
            if(fabs(anglePitch-mountAngle)<=5)
            {
                pointInfoVec[i].groundLabel=1;
                pointInfoVec[i+1].groundLabel=1;
            }
            if(fabs(laserMsgInfo->points[i].x)<0.01)
            {
                pointInfoVec[i].point000=1;
                pointInfoVec[i+1].point000=1;                
            }
        }
        
    }
    //点云分割
    void pointCloudSegmentation()
    {
        for(int i=0;i<cloudPointSize;i++)
        {
            if((pointInfoVec[i].groundLabel==0)&&(pointInfoVec[i].point000==0))
            {
                pointWithoutGround->push_back(laserMsgInfo->points[i]);
            }
            allPointCloud->push_back(laserMsgInfo->points[i]);
        }
    }

    //下采样
    void downSampling(const pcl::PointCloud<PointType>::Ptr &pcSamplePtr)
    {
        tt.tic(); //before voxel grid
        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<PointType> vg;
        vg.setInputCloud (pcSamplePtr);
        float leaf=0.1f;// 单位：m
        vg.setLeafSize(leaf,leaf,leaf);
        vg.setDownsampleAllData(true); //work on all points?
        vg.filter(*cloudFilter);
        std::cout << "voxel grid done,cost " << tt.toc() << " ms," \
            << pcSamplePtr->points.size() << " -> " << cloudFilter->points.size() << " points"<< std::endl;
    }
    
    // 点云聚类 调用库实现
    void pointCloudClustering(pcl::PointCloud<PointType>::Ptr &pcClusterPtr)
    {
        downSampling(pcClusterPtr);
        tt.tic(); //before voxel grid       

        pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
        tree->setInputCloud(cloudFilter);

        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance(0.5);//设置近邻搜索的搜索半径,e.g. 0.5
        ec.setMinClusterSize(200);//设置一个聚类需要的最少点数
        ec.setMaxClusterSize(cloudFilter->size()); //设置一个聚类需要的最大点数目
        ec.setSearchMethod(tree); //设置点云的搜索机制
        ec.setInputCloud(cloudFilter);//设置原始点云
        ec.extract(clusterIndices);//从点云中提取聚类
        std::cout << "cluster done,cost " << tt.toc() << " ms," << clusterIndices.size() << " clusters" << std::endl;

        std::cout<<clusterIndices[0].indices.size()<<std::endl;
        std::cout<<clusterIndices[1].indices.size()<<std::endl;

        for(int j=0;j<clusterIndices[1].indices.size();j++)
        {
            clusterPointCloud->push_back(cloudFilter->points[clusterIndices[1].indices[j]]);
        }      
       
    }
    //粗略计算（找ABC三点）
    void calculateVolume()
    {
        tt.tic();
        sort(clusterPointCloud->begin(),clusterPointCloud->end(),cmp_x());
        float minX=clusterPointCloud->points[0].x;
        float maxX=clusterPointCloud->points[clusterPointCloud->size()-1].x;

        sort(clusterPointCloud->begin(),clusterPointCloud->end(),cmp_y());
        float minY=clusterPointCloud->points[0].y;
        float maxY=clusterPointCloud->points[clusterPointCloud->size()-1].y;

        sort(clusterPointCloud->begin(),clusterPointCloud->end(),cmp_z());
        float minZ=clusterPointCloud->points[0].z;
        float maxZ=clusterPointCloud->points[clusterPointCloud->size()-1].z;

        double coneVolume3=(maxX-minX)*(maxX-minX)*PI*(maxZ-minZ)/3;
        std::cout<<"计算体积时间"<<tt.toc()<<"ms"<<std::endl;
        std::cout<<"3coneVolume"<<coneVolume3<<std::endl;


    }
    // 精细计算（切片法）
    void calculater()
    {
        double coneVolume=0.0;
        double coneVolumeSlice=0.0;
        sort(clusterPointCloud->begin(),clusterPointCloud->end(),cmp_x());
        int accuracyX = cloudPointSize/100;//240个点
        std::vector<std::vector <PointType>> pointCloudSlice;
        pointCloudSlice.resize(accuracyX);
        pointCloudSlice.clear();
        // 分j片
        for(int j =0;j<100;j++)
        {
            float thetaX=0.0;
            float thetaZ=0.0;
            float thetaY=0.0;
            for(int i=accuracyX*j;i<accuracyX*(j+1);i++)
            {
                pointCloudSlice[j].push_back(clusterPointCloud->points[i]);
            }
            thetaX=fabs(pointCloudSlice[j].end()-pointCloudSlice[j].begin());
            sort(pointCloudSlice[j].begin(),pointCloudSlice[j].end(),cmp_y());
            int accuracyY = accuracyX/10;//24个点
            std::vector<std::vector<PointType>> pointCloudSliceLine;
            pointCloudSliceLine.resize(accuracyY);
            pointCloudSliceLine.clear();
            // 分k线
            for(int k=0;k<10;k++)
            {
                for(int l=accuracyY*k;l<accuracyY*(k+1);l++)
                {
                    pointCloudSliceLine[k].push_back(pointCloudSlice[j][l]);
                }
                thetaY=fabs(pointCloudSliceLine[k].end()-pointCloudSliceLine[k].begin());
                sort(pointCloudSliceLine[k].begin(),pointCloudSliceLine[k].end(),cmp_z());
                thetaZ=fabs(pointCloudSliceLine[k].end()-pointCloudSliceLine[k].begin());

                coneVolumeSlice+=thetaZ*thetaY;
            }
            coneVolume+=coneVolumeSlice*thetaX;
        }

    std::cout<<"体积为"<<coneVolume<<std::endl;

    }
    // void publishPoint()
    // {
    //     sensor_msgs::PointCloud2::Ptr temPointCloud;
    //     temPointCloud.reset(new sensor_msgs::PointCloud2() );
    //     ros::Time currentTime;
    //     currentTime = ros::Time::now();

    //     pcl::toROSMsg(*pointWithoutGround,*temPointCloud);
    //     temPointCloud->header.stamp=currentTime;
    //     temPointCloud->header.frame_id="livox_frame";        
    //     pubWithoutGround.publish(temPointCloud);

    //     pcl::toROSMsg(*allPointCloud,*temPointCloud);
    //     temPointCloud->header.stamp=currentTime;
    //     temPointCloud->header.frame_id="livox_frame";        
    //     pubAllPoint.publish(temPointCloud);        

    //     pcl::toROSMsg(*cloudFilter,*temPointCloud);
    //     temPointCloud->header.stamp=currentTime;
    //     temPointCloud->header.frame_id="livox_frame";
    //     pubClusterMsg.publish(temPointCloud);     

    //     pcl::toROSMsg(*clusterPointCloud,*temPointCloud);
    //     temPointCloud->header.stamp=currentTime;
    //     temPointCloud->header.frame_id="livox_frame";              
    //     pubcone.publish(temPointCloud);   
    // }
    void publishPoint()
    {
        sensor_msgs::PointCloud2::Ptr temPointCloud;
        temPointCloud.reset(new sensor_msgs::PointCloud2() );
        ros::Time currentTime;
        currentTime = ros::Time::now();

        pcl::toROSMsg(*pointWithoutGround,*temPointCloud);
        temPointCloud->header.stamp=currentTime;
        temPointCloud->header.frame_id="livox";        
        pubWithoutGround.publish(temPointCloud);

        pcl::toROSMsg(*allPointCloud,*temPointCloud);
        temPointCloud->header.stamp=currentTime;
        temPointCloud->header.frame_id="livox";        
        pubAllPoint.publish(temPointCloud);        

        pcl::toROSMsg(*cloudFilter,*temPointCloud);
        temPointCloud->header.stamp=currentTime;
        temPointCloud->header.frame_id="livox";
        pubClusterMsg.publish(temPointCloud);     

        pcl::toROSMsg(*clusterPointCloud,*temPointCloud);
        temPointCloud->header.stamp=currentTime;
        temPointCloud->header.frame_id="livox";              
        pubcone.publish(temPointCloud);   
    }

};

int main (int argc,char *argv[])
{
    ros::init(argc,argv,"coal_demo02");

    ScanFeature sf;

    ros::spin();

    return 0;
}