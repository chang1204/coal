#include <iostream>
#include <fstream>
#include <stdio.h>
#include <dirent.h>
#include <vector>
#include "nlohmann/json.hpp"
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

using namespace std;
using json = nlohmann::json;

void getFileName(const std::string & path,std::vector<std::string>& files )
{
    DIR* dir=opendir(path.c_str());
    if(dir == nullptr)
    {
        std::cerr << "Failed to open directory: "<< path << std::endl;
        return;
    }

    dirent* entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        if(entry->d_type == DT_DIR)
        {
            if(std::string(entry->d_name) == "." || std::string(entry->d_name) == "..")
            {
                continue;
            }
            getFileName(path + "/" + entry->d_name,files);
        }
        else
        {
            files.push_back(path + "/" + entry->d_name);
        }
    }

    closedir(dir);
}
 
int main(int argc,char** argv)
{
    vector<string> fileNames;
    int i = 0;
    std::string pkg_path = ros::package::getPath("coal_demo");
    YAML::Node config = YAML::LoadFile(pkg_path + "/yaml/coal.yaml");
    string input_path = config["json2txt"]["input_path"].as<std::string>();
    string output_path = config["json2txt"]["output_path"].as<std::string>();
    
    getFileName(input_path,fileNames);
    for(const auto &ph : fileNames)
    {
        //不带路径的文件名
		string::size_type iPos = ph.find_last_of("/") + 1;
		string filename = ph.substr(iPos, input_path.length() - iPos);

        std::string extension = ph.substr(ph.find_last_of(".") + 1);
        if(extension == "json")
        {
            
            std::ifstream ifs(ph);
            cout<<"输入文件："<<filename.substr(0,6)<<endl;
            json j;
            //输入内容
            ifs >> j;
            std::string name = j["objects"][0]["name"];

            // std::double_t length = j["objects"][0]["dimensions"]["length"];
            // std::double_t width = j["objects"][0]["dimensions"]["width"];
            std::double_t height = j["objects"][0]["dimensions"]["height"];

            std::double_t z = j["objects"][0]["centroid"]["z"];
            std::double_t x = j["objects"][0]["centroid"]["x"];
            // std::double_t y = j["objects"][0]["centroid"]["y"];

            // std::double_t rx = j["objects"][0]["rotations"]["x"];
            // std::double_t ry = j["objects"][0]["rotations"]["y"];
            // std::double_t rz = j["objects"][0]["rotations"]["z"];
            std::string txtpath;
            txtpath = output_path + filename.substr(0,6);

            std::string txtfile = txtpath.substr(0,txtpath.find_last_of(".")) + ".txt";
            std::ofstream ofs(txtfile);
            
            double num_z = z - (height) / 2 + 0.806;
            double num_x = x + 0.4;
            cout<<"输出文件"<<txtfile<<endl;
            ofs<<name<<" "<<"0"<<" "<<"0"<<" "<<"0"<<" "<<"0"<<" "<<"0"<<" "<<"0"<<" "<<"0"<<" "<<
            j["objects"][0]["dimensions"]["height"]<<" "<<
            j["objects"][0]["dimensions"]["width"]<<" "<<
            j["objects"][0]["dimensions"]["length"]<<" "<<
            num_x <<" "<<
            j["objects"][0]["centroid"]["y"]<<" "<<
            num_z <<" "<<
            "1.57079633";
            ofs.close();
            ifs.close();
        }
    }
    return 0;
}