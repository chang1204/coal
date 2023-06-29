#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <string>

struct Point3D {
    float x;
    float y;
    float z;
    float I;
};

int main() {

    std::string pkg_path = ros::package::getPath("coal_demo");
	YAML::Node config = YAML::LoadFile(pkg_path + "/yaml/coal.yaml");
	std::string binName = config["watch_bin"]["input_bin"].as<std::string>();

    std::ifstream file(binName, std::ios::binary);
    if (!file) {
        std::cout << "无法打开文件" << std::endl;
        return 1;
    }

    // 获取文件大小
    file.seekg(0, std::ios::end);
    int fileSize = file.tellg();
    file.seekg(0, std::ios::beg);
    std::cout<<fileSize<<std::endl;
    // 计算点云数量
    int numPoints = fileSize / sizeof(Point3D);
    std::cout<<numPoints<<std::endl;
    // 读取点云数据
    Point3D* pointCloud = new Point3D[numPoints];
    file.read(reinterpret_cast<char*>(pointCloud), fileSize);

    // 关闭文件
    file.close();

    // 指定的三维框
    float X = 6.90549664;
    float Y = -0.71064605;
    float Z = 0.62114649;
    float L = 2.0;
    float W = 3.4;
    float H = 2.7;

    float minX = X - L/2;  // 框的最小X坐标
    float minY = Y - W/2;  // 框的最小Y坐标
    float minZ = Z - H/2;  // 框的最小Z坐标
    float maxX = X + L/2;  // 框的最大X坐标
    float maxY = Y + W/2;  // 框的最大Y坐标
    float maxZ = Z + H/2;  // 框的最大Z坐标

    // 计算指定框内的点个数
    int numPointsInsideBox = 0;
    for (int i = 0; i < numPoints; ++i) {
        if (pointCloud[i].x >= minX && pointCloud[i].x <= maxX &&
            pointCloud[i].y >= minY && pointCloud[i].y <= maxY &&
            pointCloud[i].z >= minZ && pointCloud[i].z <= maxZ) {
            numPointsInsideBox++;
        }
    }

    // 输出指定框内的点个数
    std::cout << "指定框内的点个数: " << numPointsInsideBox << std::endl;

    // 释放内存
    delete[] pointCloud;

    return 0;
}
