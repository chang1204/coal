#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <std_msgs/String.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "coal_msgs/udpControl.h"
#include <chrono>

int main(int argc , char * argv[]){
    ros::init(argc , argv , "coal_udp_pub");
    ros::NodeHandle udp_nh;
    ros::Publisher udp_pub = udp_nh.advertise<coal_msgs::udpControl>("udp_pub" , 10); 

    std::string pkg_path = ros::package::getPath("coal_communication"); 
    YAML::Node config = YAML::LoadFile(pkg_path + "yaml/coal_com.yaml");
    std::string server_address = config["DESTINATION_IP"].as<std::string>();
    char DSET_IP_ADDRESS[20];
    strcpy(DSET_IP_ADDRESS, server_address.c_str());
    uint16_t dest_port = config["DESTINATION_PORT"].as<uint16_t>();

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        ROS_ERROR("Failed to create socket");
        return -1;
    }

    /* 将套接字和IP、端口绑定 */
    struct sockaddr_in addr_serv;
    int len;
    memset(&addr_serv, 0, sizeof(addr_serv)); //每个字节都用0填充
    addr_serv.sin_family = AF_INET;                    //使用IPV4地址
    addr_serv.sin_addr.s_addr = inet_addr(DSET_IP_ADDRESS);
    addr_serv.sin_port = htons(dest_port);
    len = sizeof(addr_serv);


    coal_msgs::udpControl udp_msg;
    std_msgs::String msg;
    ros::Rate rate(1);

    while (1)
    {
        ros::spinOnce();
        //开始计时
        auto timeSatrt = std::chrono::steady_clock::now();

        udp_msg.basketControl = 0;
        udp_pub.publish(udp_msg);

        msg.data = "Hello, UDP!";

        sendto(sockfd, msg.data.c_str(), msg.data.length(), 0, (struct sockaddr*)&addr_serv, len);

        //停止计时
        auto timeEnd = std::chrono::steady_clock::now();
        double timeDuration = std::chrono::duration_cast<std::chrono::microseconds>(timeEnd - timeSatrt).count();
        printf("%s %.0f \n", "程序运行时间： ", timeDuration);
        double timeDelay = (50 - timeDuration / 1000) / 1000; // 20ms 50hz
        if (timeDelay < 0)
        {
            printf("%s \n", "程序运行时间超出规定频率50hz! ");
            exit(0);
        }
        // rostopic hz /你的话题    结果在49.5hz左右即可
        ros::Duration(timeDelay).sleep();
    }
    return 0;
}