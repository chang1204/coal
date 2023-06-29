#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

namespace fs = boost::filesystem;

int main()
{
    // 源文件夹路径和目标文件夹路径
    std::string pkg_path = ros::package::getPath("coal_demo");
    YAML::Node config = YAML::LoadFile(pkg_path + "/yaml/coal.yaml");
    fs::path source_path = config["retxt"]["input_path"].as<std::string>();
    fs::path target_path = config["retxt"]["output_path"].as<std::string>();

    // 检查源文件夹是否存在
    if (!fs::exists(source_path)) {
        std::cerr << "Source folder does not exist." << std::endl;
        return 1;
    }

    // 检查目标文件夹是否存在，不存在则创建
    if (!fs::exists(target_path)) {
        if (!fs::create_directories(target_path)) {
            std::cerr << "Failed to create target folder." << std::endl;
            return 1;
        }
    }

    // 遍历源文件夹中的所有文件
    for (auto&& entry : fs::directory_iterator(source_path)) {
        // 如果是txt文件
        if (entry.path().extension() == ".txt") {
            // 打开文件并读取内容
            std::ifstream in_file(entry.path().string());
            std::string content((std::istreambuf_iterator<char>(in_file)), std::istreambuf_iterator<char>());

            // 在内容末尾添加0
            content += " 0";

            // 构造目标文件路径
            fs::path target_file_path = target_path / entry.path().filename();

            // 写入到目标文件中
            std::ofstream out_file(target_file_path.string());
            out_file << content;
        }
    }

    return 0;
}
