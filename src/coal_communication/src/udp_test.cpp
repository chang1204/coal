#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <arpa/inet.h>
#include <sys/socket.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_publisher");
    ros::NodeHandle nh;

    // ros::Publisher pub = nh.advertise<std_msgs::UInt8MultiArray>("udp_data", 10);

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        ROS_ERROR("Failed to create socket");
        return -1;
    }

    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(5589); // Replace with desired port number
    inet_pton(AF_INET, "219.216.98.90", &(dest_addr.sin_addr)); // Replace with desired destination IP address

    std_msgs::UInt8MultiArray msg;
    msg.data.resize(sizeof(int));

    ros::Rate rate(1); // Adjust the publishing rate as needed

    while (ros::ok())
    {
        int data = 42; // Replace with your int data

        // Convert int to byte array
        memcpy(msg.data.data(), &data, sizeof(int));

        // pub.publish(msg);

        sendto(sockfd, msg.data.data(), msg.data.size(), 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
        std::cout<<"1"<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    close(sockfd);
    return 0;
}

// #include <ros/ros.h>
// #include <std_msgs/UInt8MultiArray.h>
// #include <arpa/inet.h>
// #include <sys/socket.h>

// void udpCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
// {
//     if (msg->data.size() != sizeof(int))
//     {
//         ROS_ERROR("Invalid data size");
//         return;
//     }

//     int data;
//     memcpy(&data, msg->data.data(), sizeof(int));

//     ROS_INFO("Received UDP data: %d", data);
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "udp_subscriber");
//     ros::NodeHandle nh;

//     ros::Subscriber sub = nh.subscribe("udp_data", 10, udpCallback);

//     int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
//     if (sockfd < 0)
//     {
//         ROS_ERROR("Failed to create socket");
//         return -1;
//     }

//     struct sockaddr_in addr;
//     addr.sin_family = AF_INET;
//     addr.sin_port = htons(12345); // Replace with the same port number used in the publisher
//     addr.sin_addr.s_addr = htonl(INADDR_ANY);

//     if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
//     {
//         ROS_ERROR("Failed to bind socket");
//         close(sockfd);
//         return -1;
//     }

//     ros::spin();

//     close(sockfd);
//     return 0;
// }
