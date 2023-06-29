#include "extract_node.h"

/*~~~~~~~~~~~~~~~~~~~~~~~~用户接口~~~~~~~~~~~~~~~~~~~~~~~*/
/**
 * @brief: 构造函数调用：整体调用煤堆特征提取节点功能
 * @note:
 */
ExtractNode::ExtractNode()
{
    this->SystemInit();
    ros::Rate r(100);
    while (ros::ok())
    {
        ros::spinOnce();
        this->MsgPub();
        r.sleep();
    }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~初始化~~~~~~~~~~~~~~~~~~~~~~~*/
/**
 * @brief: 系统初始化
 * @note:
 */
bool ExtractNode::SystemInit()
{
    //重置标志位
    is_find_A_ = false;
    sample_count_ = 0;
    //分配内存
    raw_pointcloud_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>());
    pretreat_pointcloud_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>());
    dump_pointcloud_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>());

    // ros消息发布接收
    pub_pretreat_ = nh_.advertise<sensor_msgs::PointCloud2>("feature_points", 10);
    pub_dump_ = nh_.advertise<sensor_msgs::PointCloud2>("dump_points", 10);
    sub_livox_ = nh_.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 10, &ExtractNode::LivoxPointCloudHandler, this);

    return true;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~回调刷新~~~~~~~~~~~~~~~~~~~~~~~*/
/**
 * @brief: livox消息接收处理
 * @note:
 * @todo: 剔除底面点
 */
void ExtractNode::LivoxPointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    TicToc t1;

    // ros消息类型转换成pcl
    // this->ConvertPointCloud2PCL(cloud_msg);
    pcl::fromROSMsg(*cloud_msg,*raw_pointcloud_);

    ROS_INFO("size: %d",raw_pointcloud_->points.size());

    //前后各3个点求曲率
    int sp = 3;
    int ep = raw_pointcloud_->points.size() - 4;

    for (int i = sp; i < ep; i++)
    {
        //计算极坐标距离
        float range = raw_pointcloud_->points[i].x * raw_pointcloud_->points[i].x + raw_pointcloud_->points[i].y * raw_pointcloud_->points[i].y + raw_pointcloud_->points[i].z * raw_pointcloud_->points[i].z;
        //计算曲率
        float diff_x = raw_pointcloud_->points[i - 3].x + raw_pointcloud_->points[i - 2].x + raw_pointcloud_->points[i - 1].x + raw_pointcloud_->points[i + 1].x + raw_pointcloud_->points[i + 2].x + raw_pointcloud_->points[i + 3].x - 6 * raw_pointcloud_->points[i].x;
        float diff_y = raw_pointcloud_->points[i - 3].y + raw_pointcloud_->points[i - 2].y + raw_pointcloud_->points[i - 1].y + raw_pointcloud_->points[i + 1].y + raw_pointcloud_->points[i + 2].y + raw_pointcloud_->points[i + 3].y - 6 * raw_pointcloud_->points[i].y;
        float diff_z = raw_pointcloud_->points[i - 3].z + raw_pointcloud_->points[i - 2].z + raw_pointcloud_->points[i - 1].z + raw_pointcloud_->points[i + 1].z + raw_pointcloud_->points[i + 2].z + raw_pointcloud_->points[i + 3].z - 6 * raw_pointcloud_->points[i].z;
        float diff = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;

        // ROS_INFO("z: %f",raw_pointcloud_->points[i]);

        //曲率大于100,且z方向曲率大于阈值的点 存入预处理点云
        if (diff / range > 5 && raw_pointcloud_->points[i].z > -0.01 && raw_pointcloud_->points[i].z < 0.0)
        {
            pretreat_pointcloud_->points.push_back(raw_pointcloud_->points[i]);
        }
    }

    //--点云聚类
    this->PointClustering();
    //--位姿解算
    this->PoseResolving();

    //采样十次算一次体积
    if (sample_count_ == 10)
    {
        //--拟合体积
        this->VolumeMatching();
        sample_count_ = 0;
    }

    printf("程序运行时间： %f ms *** \n", t1.toc());
}

/*~~~~~~~~~~~~~~~~~~~~~~~~功能函数~~~~~~~~~~~~~~~~~~~~~~~*/
// /**
//  * @brief: 消息类型转换
//  * @note:
//  */
// void ExtractNode::ConvertPointCloud2PCL(const sensor_msgs::PointCloud::ConstPtr &cloud_msg)
// {
//     //声明了一个pcl::PointCloud<pcl::PointXYZ>::Ptr的指针cloud_msg并进行初始化，pcl::PointCloud<pcl::PointXYZ>::Ptr的数据类型为boost::shared_ptr。
//     pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>());
//     // 对容器进行初始化
//     int num = cloud_msg->points.size();
//     point_cloud->points.resize(num);

//     for (unsigned int i = 0; i < num; ++i)
//     {
//         // 首先声明一个 cloud_msg第i个点的 引用
//         pcl::PointXYZ &point_tmp = point_cloud->points[i];
//         // 获取scan的第i个点的距离值
//         point_tmp.x = cloud_msg->points[i].x;
//         point_tmp.y = cloud_msg->points[i].y;
//         point_tmp.z = cloud_msg->points[i].z;
//     }

//     point_cloud->width = cloud_msg->points.size();
//     point_cloud->height = 1;
//     point_cloud->is_dense = true;                                   // 不包含 nans 点
//     pcl_conversions::toPCL(cloud_msg->header, point_cloud->header); //复制消息头

//     *raw_pointcloud_ = *point_cloud;
// }

/**
 * @brief: 点云聚类
 * @note:
 */
bool ExtractNode::PointClustering()
{
    //找到最大和最小的x,y
    int xmin_num = 1, xmax_num = 1, ymin_num = 1, ymax_num = 1;
    float temp_max_x = 0, temp_max_y = 0, temp_min_x = 0, temp_min_y = 0;
    for (size_t i = 1; i < pretreat_pointcloud_->points.size(); i++)
    {
        if (pretreat_pointcloud_->points[i].x < temp_min_x)
        {
            xmin_num = i;
            temp_min_x = pretreat_pointcloud_->points[i].x;
        }
        else if (pretreat_pointcloud_->points[i].x > temp_max_x)
        {
            xmax_num = i;
            temp_max_x = pretreat_pointcloud_->points[i].x;
        }

        if (pretreat_pointcloud_->points[i].y < temp_min_y)
        {
            ymin_num = i;
            temp_min_y = pretreat_pointcloud_->points[i].y;
        }
        else if (pretreat_pointcloud_->points[i].y > temp_max_y)
        {
            ymax_num = i;
            temp_max_y = pretreat_pointcloud_->points[i].y;
        }
    }
    // 0 - max_x-1和min_y+0.2 - max_y-0.2
    for (size_t i = 0; i < raw_pointcloud_->points.size(); i++)
    {
        if (raw_pointcloud_->points[i].x > 0 &&
            raw_pointcloud_->points[i].x < temp_max_x - 1 &&
            raw_pointcloud_->points[i].y > temp_min_y + 0.3 &&
            raw_pointcloud_->points[i].y < temp_max_y - 0.3 &&
            raw_pointcloud_->points[i].z > -0.38 &&
            raw_pointcloud_->points[i].z < 1.9)
        {
            dump_pointcloud_->points.push_back(raw_pointcloud_->points[i]);
        }
    }
    //采样次数++
    sample_count_++;

    return true;
}

/**
 * @brief: 位姿解算
 * @note:
 * @todo: 目前只输出相对的x,y 后续根据需求调整
 */
bool ExtractNode::PoseResolving()
{
    //相对x：找到 dump 中 max_x
    //相对y: 找到 dump 中的 min_y+max_y/2
    int xmin_num = 1, xmax_num = 1, ymin_num = 1, ymax_num = 1;
    // float max_x = 0, max_y = 0, min_x = 0, min_y = 0;
    for (size_t i = 1; i < dump_pointcloud_->points.size(); i++)
    {
        if (dump_pointcloud_->points[i].x < min_x)
        {
            xmin_num = i;
            min_x = dump_pointcloud_->points[i].x;
        }
        else if (dump_pointcloud_->points[i].x > max_x)
        {
            xmax_num = i;
            max_x = dump_pointcloud_->points[i].x;
        }

        if (dump_pointcloud_->points[i].y < min_y)
        {
            ymin_num = i;
            min_y = dump_pointcloud_->points[i].y;
        }
        else if (dump_pointcloud_->points[i].y > max_y)
        {
            ymax_num = i;
            max_y = dump_pointcloud_->points[i].y;
        }
    }
    std::cout << "* * * * * * * * * * * * * * * * * * * * * * * * * * * * " << std::endl;
    std::cout << "* 煤堆中心位置与雷达距离：x " << max_x << "米   y: " << (max_y + min_y) / 2 << "米 *" << std::endl;
    std::cout << "* * * * * * * * * * * * * * * * * * * * * * * * * * * *  " << std::endl;

    return true;
}

/**
 * @brief: 体积拟合
 * @note:
 * @todo: 根据yaml配置文件确定体积解算精度
 */
void ExtractNode::VolumeMatching()
{
    //求得煤堆高度差
    int zmin_num = 1, zmax_num = 1;
    // float max_z = 0, min_z = 0;
    for (size_t i = 1; i < dump_pointcloud_->points.size(); i++)
    {
        if (dump_pointcloud_->points[i].z < min_z)
        {
            zmin_num = i;
            min_z = dump_pointcloud_->points[i].z;
        }
        else if (dump_pointcloud_->points[i].z > max_z)
        {
            zmax_num = i;
            max_z = dump_pointcloud_->points[i].z;
        }
    }

    float volume = 0, area = 0;
    //以百分之1的高度差为精度拟合体积
    float dz = (max_z - min_z) / 100.0f;
    for (size_t i = 0; i < 100; i++)
    {
        for (size_t j = 0; j < dump_pointcloud_->points.size(); j++)
        {
            if (dump_pointcloud_->points[j].z - (min_z + i * dz) > 0 &&
                dump_pointcloud_->points[j].z - (min_z + i * dz) < dz)
            {
                slice_points_.push_back(dump_pointcloud_->points[j]);
            }
        }
        //按照y的大小对容器重新排序
        sort(slice_points_.begin(), slice_points_.end(), comp);

        for (size_t k = 0; k < slice_points_.size() - 1; k++)
        {
            area += abs(slice_points_[k + 1].y - slice_points_[k].y) * abs(max_x - slice_points_[k].x);
        }
        volume += area * dz;
        area = 0;
        slice_points_.clear();
    }

    std::cout << "* * * * * * * * * * * * * * * " << std::endl;
    std::cout << "* 煤堆体积： " << volume * 2 << " 立方米 *" << std::endl;
    std::cout << "* * * * * * * * * * * * * * *" << std::endl;

    //容器清空
    dump_pointcloud_->points.clear();
    return;
}

/**
 * @brief: 消息发布
 * @note:
 */
void ExtractNode::MsgPub()
{
    pretreat_pointcloud_->header = raw_pointcloud_->header;
    pcl::toROSMsg(*pretreat_pointcloud_, pretreat_pointcloud2_);
    dump_pointcloud_->header = raw_pointcloud_->header;
    pcl::toROSMsg(*dump_pointcloud_, dump_pointcloud2_);
    //发布提取的特征点云
    pub_pretreat_.publish(pretreat_pointcloud2_);
    pub_dump_.publish(dump_pointcloud2_);
}

/*~~~~~~~~~~~~~~~~~~~~~~~~main函数~~~~~~~~~~~~~~~~~~~~~~~*/
/**
 * @brief: main
 * @note:
 */
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "extract_node");
    ExtractNode en;

    return 0;
}
