#ifndef _COAL_H_
#define _COAL_H_

#include <unordered_map>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <laser_geometry/laser_geometry.h>


#include <opencv2/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <pcl/console/time.h>//计时
#include <pcl/segmentation/extract_clusters.h>


#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

// #include"tic_toc.h"

#include <limits>
#include <tf2/utils.h>
// #include <csm/csm_all.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include "tools_eigen_math.hpp"
#include"tic_toc.h"
#define USE_HASH 1
#define PI 3.14159265

using namespace std;

typedef pcl::PointXYZI  PointType;


// 平滑度结构体
struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

/*
    * A point cloud type that has "ring" channel
    */
struct PointXYZIR
{
    PCL_ADD_POINT4D//x y z 
    PCL_ADD_INTENSITY;//intensity
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,  
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (std::uint16_t, ring, ring)
)

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
//增加新的PointT类型常规操作
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D// 该点类型有4个元素
    PCL_ADD_INTENSITY;//这可以用作索引
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW// 确保new操作符对齐操作
} EIGEN_ALIGN16;// 强制SSE对齐

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)
// 而 PointTypePose指的是具备姿态角的特定点:
typedef PointXYZIRPYT  PointTypePose;
enum ePointTpye
{
        ePointNormal = 0,                      // normal points
        ePoint000 = 0x0001 << 0,               // points [0,0,0]
        ePointTooNear = 0x0001 << 1,          // points in short range
        ePointReflectivityLow = 0x0001 << 2,  // low reflectivity
        ePointReflectivityHigh = 0x0001 << 3, // high reflectivity
        ePointCircleEdge = 0x0001 << 4,       // points near the edge of circle
        ePointNan = 0x0001 << 5,               // points with infinite value
        ePOintSmallViewAngle = 0x0001 << 6,  // points with large viewed angle
};
enum E_feature_type // if and only if normal point can be labeled
{
    e_label_invalid = -1,
    e_label_unlabeled = 0,
    e_label_corner = 0x0001 << 0,
    e_label_surface = 0x0001 << 1,
    e_label_near_nan = 0x0001 << 2,
    e_label_near_zero = 0x0001 << 3,
    e_label_hight_intensity = 0x0001 << 4,
};
struct pointInfo
{
    int pointType=ePointNormal;
    int PointLabel=e_label_unlabeled;
    int   idx = 0.f;
    float raw_intensity = 0.f;
    float time_stamp = 0.0;
    float polar_angle = 0.f;
    int   polar_direction = 0;
    float polar_dis_sq2 = 0.f;
    float depth_sq2 = 0.f;
    float curvature = 0.0;
    float view_angle = 0.0;
    float sigma = 0.0;
    int groundLabel=0;
    int point000;
    int range = 0;
    Eigen::Matrix< float, 2, 1 > pt_2d_img; // project to X==1 plane
};
enum E_point_type
{
    e_pt_normal = 0,                      // normal points
    e_pt_000 = 0x0001 << 0,               // points [0,0,0]
    e_pt_too_near = 0x0001 << 1,          // points in short range
    e_pt_reflectivity_low = 0x0001 << 2,  // low reflectivity
    e_pt_reflectivity_high = 0x0001 << 3, // high reflectivity
    e_pt_circle_edge = 0x0001 << 4,       // points near the edge of circle
    e_pt_nan = 0x0001 << 5,               // points with infinite value
    e_pt_small_view_angle = 0x0001 << 6,  // points with large viewed angle
};
template < typename T >
T depth2_xyz( T x, T y, T z )
{
    return x * x + y * y + z * z;
}
template < typename T >
T dis2_xy( T x, T y )
{
    return x * x + y * y;
}

struct Pt_hasher
{
    template < typename _T >
    std::size_t operator()( const _T &k ) const
    {
        return ( ( std::hash< float >()( k.x ) ^ ( std::hash< float >()( k.y ) << 1 ) ) >> 1 ) ^ ( std::hash< float >()( k.z ) << 1 );
    }
};

struct Pt_compare
{
    //inline bool operator()( const pcl::PointXYZ& a,  const pcl::PointXYZ & b)
    template < typename _T >
    inline bool operator()( const _T &a, const _T &b )
    {
        return ( ( a.x < b.x ) || ( a.x == b.x && a.y < b.y ) || ( ( a.x == b.x ) && ( a.y == b.y ) && ( a.z < b.z ) ) );
    }

    template < typename _T >
    bool operator()( const _T &a, const _T &b ) const
    {
        return ( a.x == b.x ) && ( a.y == b.y ) && ( a.z == b.z );
    }
};
enum E_intensity_type
{
    e_I_raw = 0,
    e_I_motion_blur,
    e_I_motion_mix,
    e_I_sigma,
    e_I_scan_angle,
    e_I_curvature,
    e_I_view_angle,
    e_I_time_stamp
};
struct clusterPoint
{
    float pointX;
    float pointY;
    float pointZ;
    int pointIdx;

};

struct cmp_x{ 
    bool operator()(PointType const &p1, PointType const &p2) { 
        return p1.x < p2.x;
    }
};

struct cmp_y{ 
    bool operator()(PointType const &p1, PointType const &p2) { 
        return p1.y < p2.y;
    }
};
struct cmp_z{ 
    bool operator()(PointType const &p1, PointType const &p2) { 
        return p1.z < p2.z;
    }
};
#endif
