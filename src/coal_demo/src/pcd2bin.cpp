// #include <iostream>           
// // #include <pcl/io/pcd_io.h>      
// // #include <pcl/point_types.h>
// #include "coal.h"
// #include <string>
// #include <vector>
// #include <stdio.h>
// #include <dirent.h>
// #include <yaml-cpp/yaml.h>
// #include <ros/package.h>

// using namespace std;

// void getFileNames(const std::string& path, std::vector<std::string>& files) {
//     DIR* dir = opendir(path.c_str());
//     if (dir == nullptr) {
//         std::cerr << "Failed to open directory: " << path << std::endl;
//         return;
//     }

//     dirent* entry;
//     while ((entry = readdir(dir)) != nullptr) {
//         if (entry->d_type == DT_DIR) {
//             // Ignore "." and ".." directories
//             if (std::string(entry->d_name) == "." || std::string(entry->d_name) == "..") {
//                 continue;
//             }
//             // Recursively get files in subdirectory
//             getFileNames(path + "/" + entry->d_name, files);
//         } else {
//             files.push_back(path + "/" + entry->d_name);
//         }
//     }

//     closedir(dir);
// }

// void pcd2bin(string in_file, string out_file)
// {
// 	//Create a PointCloud value
// 	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

// 	//Open the PCD file
// 	if (pcl::io::loadPCDFile<pcl::PointXYZI>(in_file, *cloud) == -1)
// 	{
// 		PCL_ERROR("Couldn't read in_file\n");
// 	}
// 	//Create & write .bin file
// 	ofstream bin_file(out_file.c_str(), ios::out | ios::binary | ios::app);
// 	if (!bin_file.good()) cout << "Couldn't open " << out_file << endl;

// 	//PCD 2 BIN 
// 	for (size_t i = 0; i < cloud->points.size(); ++i)
// 	{
// 		bin_file.write((char*)&cloud->points[i].x, 3 * sizeof(float));
// 		bin_file.write((char*)&cloud->points[i].intensity, sizeof(float));

// 		//bin_file.write(0, sizeof(float));
// 	}

// 	bin_file.close();
// }
// void bin2pcd(string in_file, string out_file) {
// 	fstream input(in_file.c_str(), ios::in | ios::binary);
// 	if (!input.good()) {
// 		cerr << "Couldn't read in_file: " << in_file << endl;
// 	}

// 	pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);

// 	int i;
// 	for (i = 0; input.good() && !input.eof(); i++) {
// 		pcl::PointXYZI point;
// 		input.read((char *)&point.x, 3 * sizeof(float));
// 		// input.read((char *)&point.intensity, sizeof(float));
// 		points->push_back(point);
// 	}
// 	input.close();

// 	pcl::io::savePCDFileASCII(out_file, *points);
// }
// int main(int argc, char** argv) {

// 	vector<string> fileNames;
// 	std::string pkg_path = ros::package::getPath("coal_demo");
// 	YAML::Node config = YAML::LoadFile(pkg_path+"/yaml/coal.yaml");
// 	std::string input_path = config["pcd2bin"]["input_path"].as<std::string>();
// 	std::string output_path = config["pcd2bin"]["output_path"].as<std::string>();
// 	getFileNames(input_path, fileNames);
// 	int i = 0;
// 	for (const auto &ph : fileNames) {
// 		std::cout << "ph: "<< ph<< "\n";
// 		//不带路径的文件名
// 		string::size_type iPos = ph.find_last_of("/") + 1;
// 		string filename = ph.substr(iPos, input_path.length() - iPos);
// 		cout <<"filename: "<< filename << endl;
// 		//不带后缀的文件名
// 		string name = filename.substr(0, filename.rfind("."));

// 		std::string i_string ;
// 		if (i < 10 )
// 			i_string = "00000" + to_string(i);			
// 		else if(10<=i<100)
// 			i_string = "0000" + to_string(i);	
// 		else
// 			i_string = "000" + to_string(i);
// 		//记得将这里的路径改为自己转换后的点云要存储的文件路径
// 		//pcd格式的点云转成bin格式的，对于不含强度信息的点云会提示缺少强度intensity信息，但还是可以转换成功
// 		pcd2bin(ph, output_path + i_string + ".bin");
// 		i++;
// 		cout <<"name: "<< i_string << endl;
// 		//bin格式的点云转成pcd格式的,使用方法注释上面那行，取消注释下面这行。
// 		//bin2pcd(ph, "E:/DataSet/PCD/" + name + ".pcd");
// 	}
// 	return 0;
// }



#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
    // if (argc != 3) {
    //     std::cerr << "Usage: pcd2bin <source_folder> <target_folder>" << std::endl;
    //     return -1;
    // }

    // std::string source_folder(argv[1]);
    // std::string target_folder(argv[2]);

	std::string pkg_path = ros::package::getPath("coal_demo");
	YAML::Node config = YAML::LoadFile(pkg_path+"/yaml/coal.yaml");
	std::string source_folder = config["pcd2bin"]["input_path"].as<std::string>();
	std::string target_folder = config["pcd2bin"]["output_path"].as<std::string>();

    if (!fs::exists(source_folder) || !fs::is_directory(source_folder)) {
        std::cerr << "Error: " << source_folder << " does not exist or is not a directory." << std::endl;
        return -1;
    }

    if (!fs::exists(target_folder) || !fs::is_directory(target_folder)) {
        std::cerr << "Error: " << target_folder << " does not exist or is not a directory." << std::endl;
        return -1;
    }

    int file_index = 0;
    for (fs::directory_iterator it(source_folder); it != fs::directory_iterator(); ++it) {
        if (!fs::is_regular_file(it->path()) || it->path().extension() != ".pcd") {
            continue;
        }

		// std::string i_string ;
		// if (file_index < 10 )
		// 	i_string = "00000" + std::to_string(file_index);			
		// else if(10<=file_index<100)
		// 	i_string = "0000" + std::to_string(file_index);	
		// else
		// 	i_string = "000" + std::to_string(file_index);
        // std::string filename = i_string + ".bin";

        std::string source_path = it->path().string();
        std::string file_name = source_path.substr(source_path.find_last_of("/") + 1);
        std::string new_name = file_name.substr(0,file_name.find_last_of("."));
        
        // if(new_name.size()>6){
        //     new_name = new_name.substr(1,7);
        // }

        std::string target_path = (fs::path(target_folder) / new_name).string() + ".bin";
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile(source_path, *cloud);

        std::ofstream outfile(target_path, std::ios::out | std::ios::binary);
        if (!outfile.is_open()) {
            std::cerr << "Error: failed to open " << target_path << " for writing." << std::endl;
            return -1;
        }

        for (size_t i = 0; i < cloud->size(); ++i) {
            const pcl::PointXYZI& pt = cloud->at(i);
            outfile.write(reinterpret_cast<const char*>(&pt.x), sizeof(float));
            outfile.write(reinterpret_cast<const char*>(&pt.y), sizeof(float));
            outfile.write(reinterpret_cast<const char*>(&pt.z), sizeof(float));
            outfile.write(reinterpret_cast<const char*>(&pt.intensity), sizeof(float));
        }

        outfile.close();

        ++file_index;
    }

    return 0;
}
