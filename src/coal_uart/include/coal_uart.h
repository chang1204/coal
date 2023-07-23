//--ros库函数
#include "ros/ros.h"
#include <serial/serial.h>
#include <ros/package.h>
//--c++库函数
#include "bits/stdc++.h"
using namespace std;
#include <sys/timeb.h>
//参数读取
#include "../Jason/jason.hpp"
using json = nlohmann::json;
//自定义消息类型
#include "coal_msgs/sensorDataReception.h"
#include "coal_msgs/udp2uart.h"

#include "coal_msgs/keyboard2udp.h"

#define RX_LENGTH   30   //接收数据长度

#include "fontColor.h";
#include "../include/crc.h";

class coal_uart
{
  public:
    coal_uart();
    ~coal_uart();

  private:
    //--串口配置
    void uartDeviceConfig(void);
    //--信息流
    void msgFlowConfig(void);
    //--串口读取数据
    void readData(void);
    //--解析
    void analyzePackage();
    //--记录时间戳
    void unixTimeRecord(void);
    //--终端tag
    void displayTag(char str);
    void keyboardCallback(const coal_msgs::udp2uart::ConstPtr &msg);
    void uartCallback(const coal_msgs::keyboard2udp::ConstPtr &msg);

    json param;
    //--串口缓冲区
    deque<uint8_t> uartBuffer;
    //--串口配置器
    serial::Serial uartSolver;
    //--ros配置
    ros::NodeHandle n;

    // 定义CRC校验倒数两位
    uint8_t last1,last2;
    // CRC校验
    CRC uart_check;
    // --串口接收数组
    uint8_t uartRxBuffer[RX_LENGTH];

    //控制指令定时器
    ros::Timer cmdTimer;
    ros::Subscriber keyboard_sub;
    ros::Subscriber uart_sub;
    void controlCmdSendCallback(const ros::TimerEvent &event);
    coal_msgs::keyboard2udp keyboard2udp;
    coal_msgs::udp2uart udp2uart;
    coal_msgs::sensorDataReception sensorDataReception;
};