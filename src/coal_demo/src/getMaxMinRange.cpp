#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include "nlohmann/json.hpp"

int main(int argc, char**argv)
{
	std::string pkg_path = ros::package::getPath("coal_demo");
	YAML::Node config = YAML::LoadFile(pkg_path + "/yaml/coal.yaml");
	std::string fileName = config["getMaxMinRange"]["input_pcd"].as<std::string>();
	std::string binName = config["getMaxMinRange"]["input_bin"].as<std::string>();
    std::string jsonName = config["getMaxMinRange"]["input_json"].as<std::string>();

    std::ifstream ifs(jsonName);
    nlohmann::json j;
    ifs >> j;

    std::double_t L = j["objects"][0]["dimensions"]["length"];
    std::double_t W = j["objects"][0]["dimensions"]["width"];
    std::double_t H = j["objects"][0]["dimensions"]["height"];
    std::double_t Z = j["objects"][0]["centroid"]["z"];
    std::double_t X = j["objects"][0]["centroid"]["x"];
    std::double_t Y = j["objects"][0]["centroid"]["y"];

    ifs.close();

	//读入点云
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	std::cout<<"输入pcd："<<fileName<<std::endl;
	
	pcl::io::loadPCDFile(fileName, *cloud);
	
	pcl::PointXYZI minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);  //获取点云最大最小值

	std::cout << "max x is:" << maxPt.x << std::endl <<
		"max y is:" << maxPt.y << std::endl <<
		"max z is:" << maxPt.z << std::endl <<
		"min x is:" << minPt.x << std::endl <<
		"min y is:" << minPt.y << std::endl <<
		"min z is:" << minPt.z << std::endl;

    float minX = X - L/2;  // 框的最小X坐标
    float minY = Y - W/2;  // 框的最小Y坐标
    float minZ = Z - H/2;  // 框的最小Z坐标
    float maxX = X + L/2;  // 框的最大X坐标
    float maxY = Y + W/2;  // 框的最大Y坐标
    float maxZ = Z + H/2;  // 框的最大Z坐标

    // 计算指定框内的点个数
    std::cout<<cloud->size()<<std::endl;
    int numPointsInsideBox = 0;
    for (int i = 0; i < cloud->size(); ++i) {
        if (cloud->points[i].x >= minX && cloud->points[i].x <= maxX &&
            cloud->points[i].y >= minY && cloud->points[i].y <= maxY &&
            cloud->points[i].z >= minZ && cloud->points[i].z <= maxZ) {
            numPointsInsideBox++;
        }
    }
    std::cout << "指定框内的点个数: " << numPointsInsideBox << std::endl;
	return 0;
}

