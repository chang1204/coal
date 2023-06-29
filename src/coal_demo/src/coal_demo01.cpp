/*
 * @Author: changsaier changsaier@icloud.com
 * @Date: 2022-11-01 10:26:06
 * @LastEditors: changsaier changsaier@icloud.com
 * @LastEditTime: 2022-11-04 20:52:55
 * @FilePath: /catkin_chang/src/icp_demo01/src/Coal.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "coal.h"
#include "glog/logging.h"
 
class ScanFeature
{
private:
    ros::NodeHandle nh;
    // ros::NodeHandle pnh;
    ros::Subscriber subLaserScan;

    ros::Publisher m_pub_pc_livox_corners, m_pub_pc_livox_surface, m_pub_pc_livox_full;
    ros::Publisher m_pub_pc_livox_full_all;
    ros::Publisher m_pub_pc_livox_mount;
    ros::Publisher m_pub_pc_livox_mount_all;
    sensor_msgs::PointCloud2  temp_out_msg;


    std::vector<pointInfo> pointInfoVec;
    std::vector<PointType> rawPointVec;

    std::vector<pcl::PointCloud<PointType> >   m_map_pointcloud_full_vec;
    std::vector<pcl::PointCloud<PointType> >   m_map_pointcloud_surface_vec;
    std::vector<pcl::PointCloud<PointType> >   m_map_pointcloud_corner_vec;
    std::vector<pcl::PointCloud<PointType> >   m_map_pointcloud_mount_vec;
    
    std::vector<PointType> pointCloudMound;
    pcl::PointCloud<PointType>::Ptr livox_corners_all=boost::shared_ptr<pcl::PointCloud<PointType>>( new pcl::PointCloud<PointType>() );
    pcl::PointCloud<PointType>::Ptr livox_surface_all=boost::shared_ptr<pcl::PointCloud<PointType>>( new pcl::PointCloud<PointType>() );
    pcl::PointCloud<PointType>::Ptr livox_full_all=boost::shared_ptr<pcl::PointCloud<PointType>>( new pcl::PointCloud<PointType>() );
    pcl::PointCloud<PointType>::Ptr livox_mount_all=boost::shared_ptr<pcl::PointCloud<PointType>>( new pcl::PointCloud<PointType>() );

    // pcl::PointCloud<PointType>::Ptr mountPoint=boost::shared_ptr<pcl::PointCloud<PointType>>( new pcl::PointCloud<PointType>() );
    pcl::PointCloud<PointType> mountPoint;

    pcl::PointCloud<PointType> laserMsgInfo;
    pcl::VoxelGrid<PointType> m_voxel_filter_for_surface;
    pcl::VoxelGrid<PointType> m_voxel_filter_for_corner;
    
#if USE_HASH
    std::unordered_map< PointType, pointInfo *, Pt_hasher, Pt_compare > m_map_pt_idx; // using hash_map
    std::unordered_map< PointType, pointInfo *, Pt_hasher, Pt_compare >::iterator m_map_pt_idx_it;
#else
    std::map< PointType, pointInfo *, Pt_compare >           m_map_pt_idx;
    std::map< PointType, pointInfo *, Pt_compare >::iterator m_map_pt_idx_it;
#endif
    E_intensity_type default_return_intensity_type = e_I_motion_blur;

    int m_odom_mode = 0; //0 = for odom, 1 = for mapping

    float m_livox_min_allow_dis = 1.0;//1
    float m_livox_min_sigma = 7e-3;
    float m_max_edge_polar_pos = 0;
    float minimum_view_angle = 10;//10度
    float thr_corner_curvature = 0.05;
    float thr_surface_curvature = 0.01;
    bool systemInit = false;
    int systemInitCount=0;
    int m_laser_scan_number = 64;//激光线束 64
    int   m_input_points_size;
    float m_time_internal_pts = 1.0e-5; // 10us = 1e-5
    double m_current_time;
    double m_last_maximum_time_stamp;
    double m_first_receive_time = -1;

    size_t laserCloudSize;
    float mountAngle = 0;
public:

    ScanFeature():
    nh("~")
    {
        // 初始化
        initParam();        
        // \033[1;32m，\033[0m 终端显示成绿色
        ROS_INFO_STREAM("\033[1;32m----> PLICP odometry started.\033[0m");
        // 接收
        subLaserScan=nh.subscribe<sensor_msgs::PointCloud> ("/scan",1,&ScanFeature::laserScanCall,this);

        // 发布
        m_pub_pc_livox_corners = nh.advertise<sensor_msgs::PointCloud2>( "/pc2_corners", 10000 );
        m_pub_pc_livox_surface = nh.advertise<sensor_msgs::PointCloud2>( "/pc2_surface", 10000 );
        m_pub_pc_livox_full = nh.advertise<sensor_msgs::PointCloud2>( "/pc2_full", 10000 );
        m_pub_pc_livox_full_all = nh.advertise<sensor_msgs::PointCloud2>( "/pc2_full_all", 10000 );
        m_pub_pc_livox_mount = nh.advertise<sensor_msgs::PointCloud2>( "/pc2_mount", 10000 );
        m_pub_pc_livox_mount_all = nh.advertise<sensor_msgs::PointCloud2>( "/pc2_mount_all", 10000 );
        


    }
public:
    void laserScanCall(const sensor_msgs::PointCloudConstPtr &laserMsg)
    {
        if(!systemInit)
        {
            systemInitCount++;
            if(systemInitCount>20)
            {
                systemInit=true;
            }
            else
             return;
        }

        sensor_msgs::PointCloud2 PC;
        sensor_msgs::convertPointCloudToPointCloud2(*laserMsg,PC);
        pcl::fromROSMsg(PC,laserMsgInfo);

        // laserMsgInfo=*laserMsg;
        laserCloudSize=laserMsgInfo.points.size();
        std::cout<<laserMsg->header.stamp<< " "<<laserCloudSize<< std::endl;
        // groundRemoval();
        featureAssociation();
    }
    void initParam()
    {
        m_map_pointcloud_full_vec.resize(3);
        m_map_pointcloud_surface_vec.resize(3);
        m_map_pointcloud_corner_vec.resize(3);
        m_map_pointcloud_mount_vec.resize(3);
        
    }
    // 标记地面
    void groundRemoval()
    {
        float diffX, diffY, diffZ, angle; 
        for(int i=0;i<laserMsgInfo.points.size();i++)
        {

            diffX=laserMsgInfo.points[i+1].x-laserMsgInfo[i].x;
            diffY=laserMsgInfo.points[i+1].y-laserMsgInfo[i].y;
            diffZ=laserMsgInfo.points[i+1].z-laserMsgInfo[i].z;
            angle=atan2(diffZ,sqrt(diffX*diffX+diffY*diffY))*180/PI;

            if(abs(angle-mountAngle)<=0.5)
            {
                std::cout<<angle<<endl;
                mountPoint.push_back(laserMsgInfo[i]);
                // pointInfoVec[i].mountLabel=1;
                // pointInfoVec[i+1].mountLabel=1;
            }            
        }    
        ros::Time current_time = ros::Time::now();
        pcl::toROSMsg(mountPoint,temp_out_msg);  
        temp_out_msg.header.stamp = current_time;
        temp_out_msg.header.frame_id = "livox";
        m_pub_pc_livox_mount.publish(temp_out_msg);

    }
    bool getGround(int i)
    {
        float diffX, diffY, diffZ, anglePitch,angleYaw; 
        diffX=laserMsgInfo.points[i+1].x-laserMsgInfo[i].x;
        diffY=laserMsgInfo.points[i+1].y-laserMsgInfo[i].y;
        diffZ=laserMsgInfo.points[i+1].z-laserMsgInfo[i].z;
        anglePitch=atan2(diffZ,sqrt(diffX*diffX+diffY*diffY))*180/PI;
        if(fabs(anglePitch-mountAngle)<=5)
        {
            return true;
        }
        // angleYaw=atan2(diffY,sqrt(diffX*diffX+diffZ*diffZ))*180/PI;
        // std::cout<<angleYaw<<endl;
        // if(fabs(angleYaw-mountAngle)<=0.5)
        // {
        //     return true;
        // }
        return false;  
    }
    void featureAssociation()
    {
        std::vector<pcl::PointCloud<PointType>> laserCloudScan(m_laser_scan_number);
        
        std::vector<int> scanStartInd( 1000, 0 );
        std::vector<int> scanEndInd( 1000, 0 );

        laserCloudScan=extractLaserFeature(laserMsgInfo,laserMsgInfo.header.stamp);
        // 若提取的花瓣少于5个，则return
        if ( laserCloudScan.size() <= 5 ) // less than 5 scan
        {
            return;
        }

        m_laser_scan_number = laserCloudScan.size() * 1.0;

        scanStartInd.resize( m_laser_scan_number );
        scanEndInd.resize( m_laser_scan_number );
        std::fill( scanStartInd.begin(), scanStartInd.end(), 0 );
        std::fill( scanEndInd.begin(), scanEndInd.end(), 0 );

        int piece_wise = 3;//一次点云 分为三段 减少运动畸变
        vector<float> piece_wise_start( piece_wise );
        vector<float> piece_wise_end( piece_wise );        
        for ( int i = 0; i < piece_wise; i++ )
        {
            int start_scans, end_scans;
            //分成三段 每一段的开始和结束
            start_scans = int( ( m_laser_scan_number * ( i ) ) / piece_wise );
            end_scans = int( ( m_laser_scan_number * ( i + 1 ) ) / piece_wise ) - 1;

            int end_idx = laserCloudScan[ end_scans ].size() - 1;
            piece_wise_start[ i ] = ( ( float ) find_pt_info( laserCloudScan[ start_scans ].points[ 0 ] )->idx ) / pointInfoVec.size();
            piece_wise_end[ i ] = ( ( float ) find_pt_info( laserCloudScan[ end_scans ].points[ end_idx ] )->idx ) / pointInfoVec.size();
        }
        // 获得livox_corners（角点）, livox_surface（平面）, livox_full（好点）特征点
        for ( int i = 0; i < piece_wise; i++ )
        {
            pcl::PointCloud<PointType>::Ptr livox_corners=boost::shared_ptr<pcl::PointCloud<PointType>>( new pcl::PointCloud<PointType>() );
            pcl::PointCloud<PointType>::Ptr livox_surface=boost::shared_ptr<pcl::PointCloud<PointType>>( new pcl::PointCloud<PointType>() );
            pcl::PointCloud<PointType>::Ptr livox_full=boost::shared_ptr<pcl::PointCloud<PointType>>( new pcl::PointCloud<PointType>() );
            pcl::PointCloud<PointType>::Ptr livox_mount=boost::shared_ptr<pcl::PointCloud<PointType>>( new pcl::PointCloud<PointType>() );

            //  获得livox_corners（角点）, livox_surface（平面）, livox_full（好点）特征点
            getFeature( *livox_corners, *livox_surface, *livox_full, *livox_mount, piece_wise_start[ i ], piece_wise_end[ i ] );

            m_map_pointcloud_full_vec[i] = *livox_full;
            m_map_pointcloud_corner_vec[i] = *livox_corners;
            m_map_pointcloud_surface_vec[i] = *livox_surface;
            m_map_pointcloud_mount_vec[i] = *livox_mount;
            
        }
        for ( int i = 0; i < piece_wise; i++ )
        {
            // 只发布当前雷达的特征点
            ros::Time current_time = ros::Time::now();
            pcl::PointCloud<PointType>::Ptr livox_corners=boost::shared_ptr<pcl::PointCloud<PointType>>( new pcl::PointCloud<PointType>() );
            pcl::PointCloud<PointType>::Ptr livox_surface=boost::shared_ptr<pcl::PointCloud<PointType>>( new pcl::PointCloud<PointType>() );
            pcl::PointCloud<PointType>::Ptr livox_full=boost::shared_ptr<pcl::PointCloud<PointType>>( new pcl::PointCloud<PointType>() );
            pcl::PointCloud<PointType>::Ptr livox_mount=boost::shared_ptr<pcl::PointCloud<PointType>>( new pcl::PointCloud<PointType>() );

            *livox_full = m_map_pointcloud_full_vec[ i ];
            *livox_surface = m_map_pointcloud_surface_vec[ i ];
            *livox_corners = m_map_pointcloud_corner_vec[ i ];
            *livox_mount = m_map_pointcloud_mount_vec[ i ];

            pcl::toROSMsg( *livox_full, temp_out_msg );
            temp_out_msg.header.stamp = current_time;
            temp_out_msg.header.frame_id = "livox";
            m_pub_pc_livox_full.publish( temp_out_msg );

            m_voxel_filter_for_surface.setInputCloud( livox_surface );
            m_voxel_filter_for_surface.filter( *livox_surface );
            pcl::toROSMsg( *livox_surface, temp_out_msg );
            temp_out_msg.header.stamp = current_time;
            temp_out_msg.header.frame_id = "livox";
            m_pub_pc_livox_surface.publish( temp_out_msg );

            m_voxel_filter_for_corner.setInputCloud( livox_corners );
            m_voxel_filter_for_corner.filter( *livox_corners );
            pcl::toROSMsg( *livox_corners, temp_out_msg );
            temp_out_msg.header.stamp = current_time;
            temp_out_msg.header.frame_id = "livox";
            m_pub_pc_livox_corners.publish( temp_out_msg );

            pcl::toROSMsg( *livox_mount, temp_out_msg );
            temp_out_msg.header.stamp = current_time;
            temp_out_msg.header.frame_id = "livox";
            m_pub_pc_livox_mount.publish( temp_out_msg );

            *livox_mount_all += m_map_pointcloud_mount_vec[ i ];

            *livox_full_all += m_map_pointcloud_full_vec[ i ];

        }   
            ros::Time current_time = ros::Time::now();

            pcl::toROSMsg( *livox_mount_all, temp_out_msg );
            temp_out_msg.header.stamp = current_time;
            temp_out_msg.header.frame_id = "livox";
            m_pub_pc_livox_mount_all.publish( temp_out_msg );            
            
            pcl::toROSMsg( *livox_full_all, temp_out_msg );
            temp_out_msg.header.stamp = current_time;
            temp_out_msg.header.frame_id = "livox";
            m_pub_pc_livox_full_all.publish( temp_out_msg );    
            
            
    }
    std::vector<pcl::PointCloud<PointType>> extractLaserFeature(pcl::PointCloud<PointType> laserIn,double timeStamp =-1)
    {
        assert(timeStamp >= 0.0);
        // 若当前时间比上次最大时间小，则认为两次同一个时间，则赋值
        if(timeStamp <= 0.0000001 || (timeStamp < m_last_maximum_time_stamp) ) // old firmware, without timestamp
        {
            m_current_time = m_last_maximum_time_stamp;
        }
        else
        {
          m_current_time = timeStamp - m_first_receive_time;
        }
        if ( m_first_receive_time <= 0 )
        {
            m_first_receive_time = timeStamp;
        }
        std::vector< pcl::PointCloud< PointType >> laserCloudScans;
        std::vector< float > scan_id_index;
        laserCloudScans.clear();
        m_map_pt_idx.clear();
        rawPointVec.clear();
        pointInfoVec.clear();
        // 标记点 分点云簇
        int clutter_size = projection_scan_3d_2d( laserIn, scan_id_index );
        computeFeature();
        if(clutter_size==0)
        {
            return laserCloudScans;
        }
        else{
            splitLaserScan( clutter_size, laserIn, scan_id_index, laserCloudScans );
            return laserCloudScans;            
        }
    }
    void getFeature(pcl::PointCloud<PointType> &pc_corners,pcl::PointCloud< PointType > &pc_surface, pcl::PointCloud< PointType > &pc_full_res,pcl::PointCloud< PointType > &pc_mount_res,float minimum_blur = 0.0, float maximum_blur = 0.3 )
    {
        int corner_num = 0;
        int surface_num = 0;
        int mount_num = 0;
        int full_num = 0;
        pc_corners.resize( pointInfoVec.size() );
        pc_surface.resize( pointInfoVec.size() );
        pc_full_res.resize( pointInfoVec.size() );
        pc_mount_res.resize(pointInfoVec.size());
        float maximum_idx = maximum_blur * pointInfoVec.size();
        float minimum_idx = minimum_blur * pointInfoVec.size();
        int pt_critical_rm_mask = e_pt_000 | e_pt_nan | e_pt_too_near;
        for ( size_t i = 0; i < pointInfoVec.size(); i++ )
        {
            if ( pointInfoVec[ i ].idx > maximum_idx ||
                 pointInfoVec[ i ].idx < minimum_idx )
                continue;

            if ( ( pointInfoVec[ i ].pointType & pt_critical_rm_mask ) == 0 )
            {
                if ( pointInfoVec[ i ].pointType & e_label_corner )
                {
                    if ( pointInfoVec[ i ].pointType != e_pt_normal )
                        continue;
                    if ( pointInfoVec[ i ].depth_sq2 < std::pow( 30, 2 )  )//30^2
                    {
                        std::cout<<"1"<<endl;
                        pc_corners.points[ corner_num ] = rawPointVec[ i ];
                        // set_intensity( pc_corners.points[ corner_num ], e_I_motion_blur );
                        pc_corners.points[ corner_num ].intensity = pointInfoVec[ i ].time_stamp;
                        corner_num++;
                    }
                }
                if ( pointInfoVec[ i ].pointType & e_label_surface )
                {
                    if ( pointInfoVec[ i ].depth_sq2 < std::pow( 1000, 2 ) )//1000^2
                    {
                        pc_surface.points[ surface_num ] = rawPointVec[ i ];
                        pc_surface.points[ surface_num ].intensity = float(pointInfoVec[ i ].time_stamp);
                        // set_intensity( pc_surface.points[ surface_num ], e_I_motion_blur );
                        surface_num++;
                    }
                }
                if(pointInfoVec[i].groundLabel==0)
                {
                    pc_mount_res.points[mount_num]=rawPointVec[i];
                    pc_mount_res.points[mount_num].intensity=float(pointInfoVec[ i ].time_stamp);
                    mount_num++;
                }
            }
            pc_full_res.points[ full_num ] = rawPointVec[ i ];
            pc_full_res.points[ full_num ].intensity = pointInfoVec[ i ].time_stamp;
            full_num++;
        }

        std::cout<<corner_num<<" "<<surface_num<<" "<<full_num<<" " <<mount_num<<endl;
        pc_corners.resize(corner_num);
        pc_surface.resize(surface_num);
        pc_full_res.resize(full_num);
        pc_mount_res.resize(mount_num);
    }
    template < typename T >
    pointInfo *find_pt_info(const T & pt )
    {
        m_map_pt_idx_it = m_map_pt_idx.find( pt );
        if ( m_map_pt_idx_it == m_map_pt_idx.end() )
        {
            assert( m_map_pt_idx_it != m_map_pt_idx.end() ); // else, there must be something error happened before.
        }
        return m_map_pt_idx_it->second;
    }
        
    template <typename T>
    void splitLaserScan(const int clutter_size,const pcl::PointCloud<T> &laserSacn,
            const std::vector<float>&scan_id_index,
            std::vector<pcl::PointCloud<PointType>> &laserCloudScan)
    {
        std::vector<std::vector<int>> pointMask;
        laserCloudScan.resize(clutter_size);
        pointMask.resize(clutter_size);
        PointType point;
        int scan_idx = 0;
        //遍历所有点云，将在projection_scan_3d_2d()得到的 scan_id_index 转换成 scan_idx 表示
        for ( unsigned int i = 0; i < laserSacn.size(); i++ )
        {
            point = laserSacn.points[ i ];
            if ( i > 0 && ( ( scan_id_index[ i ] ) != ( scan_id_index[ i - 1 ] ) ) )
            {
                scan_idx = scan_idx + 1;
                pointMask[ scan_idx ].reserve( 5000 );
            }
            //将相同角度的点放在一起 
            laserCloudScan[ scan_idx ].push_back( point );
            pointMask[ scan_idx ].push_back( pointInfoVec[ i ].pointType );
        }
        laserCloudScan.resize(scan_idx);
        int remove_point_pt_type = e_pt_000 |
                                   e_pt_too_near |
                                   e_pt_nan  
                                //    e_pt_circle_edge
                                   ;
        int scan_avail_num = 0;
        std::vector< pcl::PointCloud< PointType > > res_laser_cloud_scan;
        //遍历花瓣
        for ( unsigned int i = 0; i < laserCloudScan.size(); i++ )
        {
            scan_avail_num = 0;
            pcl::PointCloud< PointType > laser_clour_per_scan;
            for ( unsigned int idx = 0; idx < laserCloudScan[ i ].size(); idx++ )
            {
                if ( ( pointMask[ i ][ idx ] & remove_point_pt_type ) == 0 )
                {
                    if ( laserCloudScan[ i ].points[ idx ].x == 0 )
                    {
                        assert( laserCloudScan[ i ].points[ idx ].x != 0 );
                        continue;
                    }
                    auto temp_pt = laserCloudScan[ i ].points[ idx ];
                    //改变强度 当前的索引占当前帧所有点的比例
                    // set_intensity( temp_pt, default_return_intensity_type );
                    laser_clour_per_scan.points.push_back(temp_pt);
                    scan_avail_num++;
                }
            }
            //printf(" %d|%d number of point in this scan  = %d ------------------\r\n ", i, laserCloudScans.size(), scan_avail_num);
            if(scan_avail_num)
            {
                res_laser_cloud_scan.push_back(laser_clour_per_scan);
            }
        }
        laserCloudScan= res_laser_cloud_scan;

    }
    int projection_scan_3d_2d(pcl::PointCloud<PointType> &laserIn,std::vector< float > &scan_id_index)
    {
        unsigned int pointSize=laserIn.size();
        pointInfoVec.clear();
        pointInfoVec.resize( pointSize );
        rawPointVec.resize( pointSize );
        std::vector< int > edge_idx;
        std::vector< int > split_idx;
        std::vector< int > zero_idx;
        m_input_points_size = 0;
        m_map_pt_idx.clear();
        m_map_pt_idx.reserve( pointSize );
        scan_id_index.resize( pointSize );

        for(unsigned int idx=0;idx<pointSize;idx++)
        {
            m_input_points_size++;
            pointInfo *pt_info = &pointInfoVec[ idx ];
            m_map_pt_idx.insert( std::make_pair( laserIn.points[ idx ], pt_info ) );
            rawPointVec[ idx ] = laserIn.points[ idx ];
            pt_info->raw_intensity = laserIn.points[ idx ].intensity;
            pt_info->idx = idx;
            pt_info->time_stamp = m_current_time + ( ( float ) idx ) * m_time_internal_pts;
            m_last_maximum_time_stamp = pt_info->time_stamp;

            
            if ( !std::isfinite( laserIn.points[ idx ].x ) ||
                 !std::isfinite( laserIn.points[ idx ].y ) ||
                 !std::isfinite( laserIn.points[ idx ].z ) )
            {
                addMaskPoint( pt_info, e_pt_nan );
                continue;
            }
            if ( laserIn.points[ idx ].x == 0 )
            {
                if ( idx == 0 )
                {
                    pt_info->pt_2d_img << 0.01, 0.01;
                    pt_info->polar_dis_sq2 = 0.0001;
                    addMaskPoint( pt_info, e_pt_000 );
                    //return 0;
                }
                else
                {
                    pt_info->pt_2d_img = pointInfoVec[ idx - 1 ].pt_2d_img;
                    pt_info->polar_dis_sq2 = pointInfoVec[ idx - 1 ].polar_dis_sq2;
                    addMaskPoint( pt_info, e_pt_000 );
                    continue;
                }
            }  
            m_map_pt_idx.insert( std::make_pair( laserIn.points[ idx ], pt_info ) );
            pt_info->depth_sq2 = depth2_xyz( laserIn.points[ idx ].x, laserIn.points[ idx ].y, laserIn.points[ idx ].z );

            pt_info->pt_2d_img << laserIn.points[ idx ].y / laserIn.points[ idx ].x, laserIn.points[ idx ].z / laserIn.points[ idx ].x;
            pt_info->polar_dis_sq2 = dis2_xy( pt_info->pt_2d_img( 0 ), pt_info->pt_2d_img( 1 ) );

            if(getGround(idx))
            {
                pt_info->groundLabel=1;
            }

            //估计强度，在投影平面上，距中心点越远的点强度小的可能性越大
            if ( pt_info->depth_sq2 < m_livox_min_allow_dis * m_livox_min_allow_dis ) // to close
            {
                //screen_out << "Add mask, id  = " << idx << "  type = e_too_near" << endl;
                addMaskPoint( pt_info, e_pt_too_near );
            }
            
            pt_info->sigma = pt_info->raw_intensity / pt_info->polar_dis_sq2;
            // 反射率   
            if ( pt_info->sigma < m_livox_min_sigma )
            {
                //screen_out << "Add mask, id  = " << idx << "  type = e_reflectivity_low" << endl;
                addMaskPoint( pt_info, e_pt_reflectivity_low );
            }

            if ( pt_info->polar_dis_sq2 > m_max_edge_polar_pos )
            {
                addMaskPoint( pt_info, e_pt_circle_edge, 2 );
            }   
            // Split scans
            if ( idx >= 1 )
            {
                float dis_incre = pt_info->polar_dis_sq2 - pointInfoVec[ idx - 1 ].polar_dis_sq2;
                // 远离零移动
                if ( dis_incre > 0 ) // far away from zero
                {
                    pt_info->polar_direction = 1;
                }
                // 向零移动
                if ( dis_incre < 0 ) // move toward zero
                {
                    pt_info->polar_direction = -1;
                }
                // 通过点云 远离中心还是靠近中心 划分点云花瓣
                if ( pt_info->polar_direction == -1 && pointInfoVec[ idx - 1 ].polar_direction == 1 )
                {
                    if ( edge_idx.size() == 0 || ( idx - split_idx[ split_idx.size() - 1 ] ) > 50 )
                    {
                        split_idx.push_back( idx );
                        edge_idx.push_back( idx );
                        continue;
                    }
                }

                if ( pt_info->polar_direction == 1 && pointInfoVec[ idx - 1 ].polar_direction == -1 )
                {
                    if ( zero_idx.size() == 0 || ( idx - split_idx[ split_idx.size() - 1 ] ) > 50 )
                    {
                        split_idx.push_back( idx );

                        zero_idx.push_back( idx );
                        continue;
                    }
                }
            }
        }
        split_idx.push_back( pointSize - 1 );

        int   val_index = 0;
        int   pt_angle_index = 0;
        float scan_angle = 0;
        int   internal_size = 0;

        if( split_idx.size() < 6) // minimum 3 petal of scan.
            return 0;
        
        for ( int idx = 0; idx < ( int ) pointSize; idx++ )
        {
            if ( val_index < split_idx.size() - 2 )
            {
                if ( idx == 0 || idx > split_idx[ val_index + 1 ] )
                {
                    if ( idx > split_idx[ val_index + 1 ] )
                    {
                        val_index++;
                    }

                    internal_size = split_idx[ val_index + 1 ] - split_idx[ val_index ];

                    if ( pointInfoVec[ split_idx[ val_index + 1 ] ].polar_dis_sq2 > 10000 )
                    {
                        pt_angle_index = split_idx[ val_index + 1 ] - ( int ) ( internal_size * 0.20 );
                        scan_angle = atan2( pointInfoVec[ pt_angle_index ].pt_2d_img( 1 ), pointInfoVec[ pt_angle_index ].pt_2d_img( 0 ) ) * 57.3;
                        scan_angle = scan_angle + 180.0;
                    }
                    else
                    {
                        pt_angle_index = split_idx[ val_index + 1 ] - ( int ) ( internal_size * 0.80 );
                        scan_angle = atan2( pointInfoVec[ pt_angle_index ].pt_2d_img( 1 ), pointInfoVec[ pt_angle_index ].pt_2d_img( 0 ) ) * 57.3;
                        scan_angle = scan_angle + 180.0;
                    }
                }
            }
            pointInfoVec[ idx ].polar_angle = scan_angle;
            scan_id_index[ idx ] = scan_angle;
        }
        return split_idx.size() - 1;       
    }
    void computeFeature()
    {
        unsigned int pointSize=rawPointVec.size();
        size_t curvatureNum=2;
        int critical_rm_point=e_pt_000 | e_pt_nan;
        float neighbor_accumulate_xyz[ 3 ] = { 0.0, 0.0, 0.0 };

        for(size_t idx=curvatureNum;idx<pointSize-curvatureNum;idx++)
        {
            if ( pointInfoVec[ idx ].pointType & critical_rm_point )
            {
                continue;
            }            

            neighbor_accumulate_xyz[ 0 ] = 0.0;
            neighbor_accumulate_xyz[ 1 ] = 0.0;
            neighbor_accumulate_xyz[ 2 ] = 0.0;
            // 空或者000标记 否则取平均
            for ( size_t i = 1; i <= curvatureNum; i++ )
            {
                if ( ( pointInfoVec[ idx + i ].pointType & e_pt_000 ) || ( pointInfoVec[ idx - i ].pointType & e_pt_000 ) )
                {
                    if ( i == 1 )
                    {
                        pointInfoVec[ idx ].PointLabel |= e_label_near_zero;//二进制 1000
                    }
                    else
                    {
                        pointInfoVec[ idx ].PointLabel = e_label_invalid;
                    }
                    break;
                }
                else if ( ( pointInfoVec[ idx + i ].pointType & e_pt_nan ) || ( pointInfoVec[ idx - i ].pointType & e_pt_nan ) )
                {
                    if ( i == 1 )
                    {
                        pointInfoVec[ idx ].PointLabel |= e_label_near_nan;//二进制0100
                    }
                    else
                    {
                        pointInfoVec[ idx ].PointLabel = e_label_invalid;
                    }
                    break;
                }
                else
                {
                    neighbor_accumulate_xyz[ 0 ] += rawPointVec[ idx + i ].x + rawPointVec[ idx - i ].x;
                    neighbor_accumulate_xyz[ 1 ] += rawPointVec[ idx + i ].y + rawPointVec[ idx - i ].y;
                    neighbor_accumulate_xyz[ 2 ] += rawPointVec[ idx + i ].z + rawPointVec[ idx - i ].z;
                }
            }

            if(pointInfoVec[ idx ].PointLabel == e_label_invalid)
            {
                continue;
            }
            // 曲率
            //求其他四个点与本个点的坐标差值，即看它们是否在空间中一条直线上，差值大则说明有角点
            neighbor_accumulate_xyz[ 0 ] -= curvatureNum * 2 * rawPointVec[ idx ].x;
            neighbor_accumulate_xyz[ 1 ] -= curvatureNum * 2 * rawPointVec[ idx ].y;
            neighbor_accumulate_xyz[ 2 ] -= curvatureNum * 2 * rawPointVec[ idx ].z;
            pointInfoVec[ idx ].curvature = neighbor_accumulate_xyz[ 0 ] * neighbor_accumulate_xyz[ 0 ] + neighbor_accumulate_xyz[ 1 ] * neighbor_accumulate_xyz[ 1 ] +
                                              neighbor_accumulate_xyz[ 2 ] * neighbor_accumulate_xyz[ 2 ];

            /*********** Compute plane angle ************/
            Eigen::Matrix< float, 3, 1 > vec_a( rawPointVec[ idx ].x, rawPointVec[ idx ].y, rawPointVec[ idx ].z );
            Eigen::Matrix< float, 3, 1 > vec_b( rawPointVec[ idx + curvatureNum ].x - rawPointVec[ idx - curvatureNum ].x,
                                                rawPointVec[ idx + curvatureNum ].y - rawPointVec[ idx - curvatureNum ].y,
                                                rawPointVec[ idx + curvatureNum ].z - rawPointVec[ idx - curvatureNum ].z );
            pointInfoVec[ idx ].view_angle = Eigen_math::vector_angle( vec_a  , vec_b, 1 ) * 57.3;

            //printf( "Idx = %d, angle = %.2f\r\n", idx,  m_pts_info_vec[ idx ].view_angle );
            if ( pointInfoVec[ idx ].view_angle > minimum_view_angle )
            {

                if( pointInfoVec[ idx ].curvature < thr_surface_curvature )
                {
                    pointInfoVec[ idx ].PointLabel |= e_label_surface;//0010
                }

                float sq2_diff = 0.1;

                if ( pointInfoVec[ idx ].curvature > thr_corner_curvature )
                {
                    //仅收纳凸出的边缘，可能因为凹进去的边缘误差较大
                    if ( pointInfoVec[ idx ].depth_sq2 <= pointInfoVec[ idx - curvatureNum ].depth_sq2 &&
                         pointInfoVec[ idx ].depth_sq2 <= pointInfoVec[ idx + curvatureNum ].depth_sq2 )
                    {
                        if ( abs( pointInfoVec[ idx ].depth_sq2 - pointInfoVec[ idx - curvatureNum ].depth_sq2 ) < sq2_diff * pointInfoVec[ idx ].depth_sq2 ||
                             abs( pointInfoVec[ idx ].depth_sq2 - pointInfoVec[ idx + curvatureNum ].depth_sq2 ) < sq2_diff * pointInfoVec[ idx ].depth_sq2 )
                            pointInfoVec[ idx ].PointLabel |= e_label_corner;
                    }
                }
            }            
        }
    }
    template < typename T >
    void set_intensity( T &pt, const E_intensity_type &i_type = e_I_motion_blur )
    {
        
        pointInfo *pt_info = find_pt_info( pt );
        switch ( i_type )
        {
        case ( e_I_raw ):
            pt.intensity = pt_info->raw_intensity;
            break;
        case ( e_I_motion_blur )://当前的索引占当前帧所有点的比例
            pt.intensity = ( ( float ) pt_info->idx ) / ( float ) m_input_points_size;
            assert( pt.intensity <= 1.0 && pt.intensity >= 0.0 );
            break;
        case ( e_I_motion_mix ):
            pt.intensity = 0.1 * ( ( float ) pt_info->idx + 1 ) / ( float ) m_input_points_size + ( int ) ( pt_info->raw_intensity );
            break;
        case ( e_I_scan_angle ):
            pt.intensity = pt_info->polar_angle;
            break;
        case ( e_I_curvature ):
            pt.intensity = pt_info->curvature;
            break;
        case ( e_I_view_angle ):
            pt.intensity = pt_info->view_angle;
            break;
        case (e_I_time_stamp):
            pt.intensity = pt_info->time_stamp;
        default:
            pt.intensity = ( ( float ) pt_info->idx + 1 ) / ( float ) m_input_points_size;
        }
        return;
    }
    // 对点云进行标记
    void addMaskPoint(pointInfo *point_Info,const E_point_type &point_Type,int neighborCount=0)
    {
        int idx=point_Info->idx;
        point_Info->pointType|=point_Type;

        if(neighborCount>0)
        {
            for(int i=-neighborCount;i<neighborCount;i++)
            {
                idx=point_Info->idx+1;
                if(i!=0&&(idx>=0)&&( idx < ( int ) pointInfoVec.size() ))
                {
                    pointInfoVec[idx].pointType|=point_Type;
                }                
            }

        }
    }



};

int main (int argc,char *argv[])
{
    ros::init(argc,argv,"coal_scan");

    ScanFeature sf;

    ros::spin();

    return 0;
}