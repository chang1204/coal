#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <chrono>
// 自定义消息类型
#include "coal_msgs/keyboard2udp.h"

coal_msgs::keyboard2udp keyboard2udp_msg;
void udpCallBack(const coal_msgs::keyboard2udp::ConstPtr& msg){
    keyboard2udp_msg.chassisSpeed1 = msg->chassisSpeed1;
    keyboard2udp_msg.chassisSpeed2 = msg->chassisSpeed2;
    keyboard2udp_msg.chassisSpeed3 = msg->chassisSpeed3;
    keyboard2udp_msg.chassisSpeed4 = msg->chassisSpeed4;
    keyboard2udp_msg.waistAngle = msg->waistAngle;
    keyboard2udp_msg.basketControl = msg->basketControl;
}

int main(int argc , char * argv[]){
    ros::init(argc , argv , "coal_udp_pub");
    ros::NodeHandle udp_nh;

    ros::Subscriber udp_sub = udp_nh.subscribe<coal_msgs::keyboard2udp>("coal_keyboard" , 10, udpCallBack);

    std::string pkg_path = ros::package::getPath("coal_communication");
    YAML::Node config = YAML::LoadFile(pkg_path + "/yaml/coal_com.yaml");
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
    memset(&addr_serv, 0, sizeof(addr_serv)); //每个字节都用0填充
    addr_serv.sin_family = AF_INET;                    //使用IPV4地址
    addr_serv.sin_addr.s_addr = inet_addr(DSET_IP_ADDRESS);
    addr_serv.sin_port = htons(dest_port);
    int len = sizeof(addr_serv);



    // std_msgs::String msg;

    std_msgs::UInt8MultiArray msg;
    msg.data.resize(sizeof(int));

    ros::Rate rate(1);

    while (ros::ok())
    {
        ros::spinOnce();
        //开始计时
        auto timeSatrt = std::chrono::steady_clock::now();

        float udp_control_array[6];
        udp_control_array[0] = keyboard2udp_msg.chassisSpeed1;
        udp_control_array[1] = keyboard2udp_msg.chassisSpeed2;
        udp_control_array[2] = keyboard2udp_msg.chassisSpeed3;
        udp_control_array[3] = keyboard2udp_msg.chassisSpeed4;
        udp_control_array[4] = keyboard2udp_msg.waistAngle;
        udp_control_array[5] = keyboard2udp_msg.basketControl;

        // 定义发送数组和发送标志位
        int send_num;
        send_num = sendto(sockfd, udp_control_array , 4 * sizeof(udp_control_array), 0, (struct sockaddr*)&addr_serv, sizeof(addr_serv));

        if (send_num < 0)
        {
            std::cout << "send to device failed" << std::endl;
        }
        else
            std::cout << "sending to another car our location"<< std::endl;

        //停止计时
        auto timeEnd = std::chrono::steady_clock::now();
        double timeDuration = std::chrono::duration_cast<std::chrono::microseconds>(timeEnd - timeSatrt).count();
        printf("%s %.0f \n", "程序运行时间： ", timeDuration);
        double timeDelay = (1000 - timeDuration / 1000) / 1000; // 20ms 50hz
        if (timeDelay < 0)
        {
            std::cout << "程序运行时间超出规定频率50hz! "<< std::endl;
            return -1;
        }
        // rostopic hz /你的话题    结果在49.5hz左右即可
        ros::Duration(timeDelay).sleep();
    }
    close(sockfd);
    return 0;
}