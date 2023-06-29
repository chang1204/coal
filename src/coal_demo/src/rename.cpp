#include <iostream>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

int main()
{
    std::string source_folder = "/home/chang/ros_project/OpenPCDet-master/data/custom/testing/velodyne/"; // 源文件夹路径
    std::string dest_folder = "/home/chang/ros_project/OpenPCDet-master/data/custom/testing/velodyne/"; // 目标文件夹路径
    int count = 0; // 计数器

    // 遍历源文件夹中的每个文件
    for (auto& file : fs::directory_iterator(source_folder))
    {
        // 获取文件扩展名
        std::string extension = fs::path(file.path()).extension().string();

        // 构造新文件名
        std::string i_string ;
        if (count < 10 )
            i_string = "00000" + std::to_string(count);			
        else if(10<=count<100)
            i_string = "0000" + std::to_string(count);	
        else
            i_string = "000" + std::to_string(count);
        std::string new_name = dest_folder + i_string + extension;

        // 重命名文件并将其复制到目标文件夹中
        fs::copy(file.path(), new_name);
        count++;
    }

    return 0;
}
