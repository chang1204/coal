#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <std_msgs/String.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "coal_msgs/udpControl.h"
#include <chrono>

void udpCallBack(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("Received UDP message: %s", msg->data.c_str());
}

int main (int argc , char* argv[]){

    ros::init(argc , argv , "coal_udp_sub");
    ros::NodeHandle udp_nh;
    ros::Subscriber udp_sub = udp_nh.subscribe<std_msgs::String>("udp_pub" , 10, udpCallBack);

    std::string pkg_path = ros::package::getPath("coal_communication");
    YAML::Node config = YAML::LoadFile(pkg_path + "/yaml/coal_com.yaml");
    uint16_t port = config["LOCAL_PORT"].as<uint16_t>();

    /* sock_fd --- socket文件描述符 创建udp套接字*/
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
        ROS_ERROR("Failed to create socket");
        return -1;
    }

    /* 将套接字和IP、端口绑定 */
    struct sockaddr_in addr_serv;
    int len;
    memset(&addr_serv, 0, sizeof(struct sockaddr_in)); //每个字节都用0填充
    addr_serv.sin_family = AF_INET;                    //使用IPV4地址
    addr_serv.sin_port = htons(port);                  //端口
    /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */
    addr_serv.sin_addr.s_addr = htonl(INADDR_ANY); //自动获取IP地址
    len = sizeof(addr_serv);
    /* 绑定socket */
    if (bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
    {
        ROS_ERROR("Failed to bind socket");
        close(sock_fd);
        return -1;
    }
    struct sockaddr_in server_addr;
    int buff[1024];
    coal_msgs::udpControl udp_msg;

    while(ros::ok()){
        float udp_control_array[6];
        int recv_num = recvfrom(sock_fd,udp_control_array,4 * sizeof(udp_control_array),0,(struct sockaddr*)&server_addr,(socklen_t*)&len);
        if(recv_num > 0){
            printf("receive success, recv_num = %d\n", recv_num);
            udp_msg.chassisSpeed1 = udp_control_array[0];
            udp_msg.chassisSpeed2 = udp_control_array[1];
            udp_msg.chassisSpeed3 = udp_control_array[2];
            udp_msg.chassisSpeed4 = udp_control_array[3];
            udp_msg.waistAngle = udp_control_array[4];
            udp_msg.basketControl = udp_control_array[5];

        }

    }
    ros::spin();
    // int recv_num = 0;
    // struct sockaddr_in addr_client;

    // while(ros::ok()){

    //     ros::spinOnce();

    //     float recv_array[6];
    //     recv_num = recvfrom(sock_fd, recv_array, sizeof(recv_array), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
    //     if (recv_num > 0)
    //     {
    //         printf("receive success, recv_num = %d\n", recv_num);
    //     }
    // }
    close(sock_fd);
    return 0;
}