#include <ros/ros.h>
#include <termios.h>
#include "coal_msgs/keyboard2udp.h"
#include "coal_msgs/udpControl.h"

int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
    int c = getchar();  // read character (blocking)
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

int main(int argc , char* argv[]){
    ros::init(argc, argv, "key_test");
    ros::NodeHandle keyboard_nh;
    ros::Publisher keyboard_pub = keyboard_nh.advertise<coal_msgs::keyboard2udp>("coal_keyboard", 10);

    ros::Rate loop_rate(10);

    coal_msgs::keyboard2udp twist_msg;
    while(ros::ok()){
        int c = getch();
        // std::cout<<c<<std::endl;
        switch (c)
        {
            case 'w':
                twist_msg.chassisSpeed1 = 1.0;
                twist_msg.chassisSpeed2 = 1.0;
                twist_msg.chassisSpeed3 = 1.0;
                twist_msg.chassisSpeed4 = 1.0;
                break;
            case 's':
                twist_msg.chassisSpeed1 = -1.0;
                twist_msg.chassisSpeed2 = -1.0;
                twist_msg.chassisSpeed3 = -1.0;
                twist_msg.chassisSpeed4 = -1.0;
                break;
            case 'a':
                twist_msg.waistAngle = 1.0;
                break;
            case 'd':
                twist_msg.waistAngle = -1.0;
                break;
            case 'u':
                twist_msg.basketControl = 1;
                break;
            case 'i':
                twist_msg.basketControl = 2;
                break;
            case 'j':
                twist_msg.basketControl = 4;
                break;
            case 'k':
                twist_msg.basketControl = 8;
                break;
            default:
                twist_msg.chassisSpeed1 = 0.0;
                twist_msg.chassisSpeed2 = 0.0;
                twist_msg.chassisSpeed3 = 0.0;
                twist_msg.chassisSpeed4 = 0.0;
                twist_msg.waistAngle = 0.0;
                twist_msg.basketControl = 0;
                break;
        }
        keyboard_pub.publish(twist_msg);
        std::cout<<twist_msg.basketControl<<std::endl;

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}