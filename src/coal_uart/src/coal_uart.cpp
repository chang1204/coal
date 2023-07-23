#include "../include/coal_uart.h"

coal_uart::coal_uart()
{
    uartDeviceConfig();
    msgFlowConfig();

    while (ros::ok)
    {
        ros::spinOnce();
        //--读取串口数据
        this->readData();
    }
    //--串口关闭
    uartSolver.close();
}

void coal_uart::uartDeviceConfig(){
    //--json参数读取配置
    ifstream(ros::package::getPath("coal_uart") + "/Jason/param.json") >> param;
    string linuxCmd;
    //--shell终端赋权限
    linuxCmd = linuxCmd + "echo " + (string)param["uartParam"].at("passWord") + " | sudo -S chmod 777 " + (string)param["uartParam"].at("usbPath");
    system(linuxCmd.c_str());

    //--串口配置
    serial::Timeout to = serial::Timeout::simpleTimeout(10); //通讯容错时间milliseconds级
    uartSolver.setTimeout(to);
    uartSolver.setPort((string)param["uartParam"].at("usbPath")); //串口路径 ls /dev/ttyUSB* 查询可使用的串口设备
    uartSolver.setBaudrate(460800);                               //串口波特率

    //--串口异常检测
    try
    {
        uartSolver.open(); //打开串口
    }
    catch (serial::IOException &e) //捕捉异常
    {
        // system("play src/roborts_uart/music/uart1.mp3");
        cerr << fontColorRed << "请检查串口号是否正确" << endl;
        exit(EXIT_FAILURE);
    }
    if (uartSolver.isOpen())
    {
        cout << (string)param["uartParam"].at("usbPath") << " 串口已经打开" << endl;
    }
    cout << "串口初始化完成" << endl;
    // system("play src/roborts_uart/music/uart0.mp3");
}

void coal_uart::msgFlowConfig(){
    // 50hz定时器
    cmdTimer = n.createTimer(ros::Duration(0.02), &coal_uart::controlCmdSendCallback, this);
    keyboard_sub = n.subscribe("udp2uart", 10, &coal_uart::keyboardCallback, this);
    uart_sub = n.subscribe("coal_keyboard", 10, &coal_uart::uartCallback, this);
}

void coal_uart::uartCallback(const coal_msgs::keyboard2udp::ConstPtr &msg){
    std::cout<<"本地键盘节点"<<std::endl;
    // udp2uart.chassisSpeed1 = msg->chassisSpeed1;
    // udp2uart.chassisSpeed2 = msg->chassisSpeed2;
    // udp2uart.chassisSpeed3 = msg->chassisSpeed3;
    // udp2uart.chassisSpeed4 = msg->chassisSpeed4;
    // udp2uart.waistAngle = msg->waistAngle;
    // udp2uart.basketControl = msg->basketControl;

}
void coal_uart::keyboardCallback(const coal_msgs::udp2uart::ConstPtr &msg){
    std::cout<<"udp2uart节点"<<std::endl;
    udp2uart.chassisSpeed1 = msg->chassisSpeed1;
    udp2uart.chassisSpeed2 = msg->chassisSpeed2;
    udp2uart.chassisSpeed3 = msg->chassisSpeed3;
    udp2uart.chassisSpeed4 = msg->chassisSpeed4;
    udp2uart.waistAngle = msg->waistAngle;
    udp2uart.basketControl = msg->basketControl;
}

void coal_uart::controlCmdSendCallback(const ros::TimerEvent &event){
    //--发送控制指令
    // displayTag('C');
    int txLength = 25;
    try{
        vector<uint8_t> controlCommand(txLength); // 25个字节的数据
        //--[0,1]帧头
        controlCommand[0] = 0x03;
        controlCommand[1] = 0xfc;
        //--[2,3]控制指令
        // chassisControl.chassisSpeed1 = 1.0f;
        // chassisControl.chassisSpeed2 = 0.0f;
        // chassisControl.chassisSpeed3 = -0.5f;
        controlCommand[2] = (int)((udp2uart.chassisSpeed1 + 4.000f) * 1000) >> 8;
        controlCommand[3] = (int)((udp2uart.chassisSpeed1 + 4.000f) * 1000) & 0xff;
        controlCommand[4] = (int)((udp2uart.chassisSpeed2 + 4.000f) * 1000) >> 8;
        controlCommand[5] = (int)((udp2uart.chassisSpeed2 + 4.000f) * 1000) & 0xff;
        controlCommand[6] = (int)((udp2uart.chassisSpeed3 + 4.000f) * 1000) >> 8;
        controlCommand[7] = (int)((udp2uart.chassisSpeed3 + 4.000f) * 1000) & 0xff;
        controlCommand[8] = (int)((udp2uart.chassisSpeed4 + 4.000f) * 1000) >> 8;
        controlCommand[9] = (int)((udp2uart.chassisSpeed4 + 4.000f) * 1000) & 0xff;

        controlCommand[10] = (int)((udp2uart.waistAngle + 180.0f) * 100) >> 8;
        controlCommand[11] = (int)((udp2uart.waistAngle + 180.0f) * 100) & 0xff;

        std::cout<<udp2uart.chassisSpeed1<<" "<<udp2uart.chassisSpeed2<<" "<<udp2uart.chassisSpeed3<<" "<<udp2uart.chassisSpeed4<<" "<<udp2uart.waistAngle<<std::endl;

        controlCommand[12] = udp2uart.basketControl >> 8;
        controlCommand[13] = udp2uart.basketControl & 0xff;
        
        controlCommand[14] = 0x00;
        controlCommand[15] = 0x00;
        controlCommand[16] = 0x00;
        controlCommand[17] = 0x00;
        controlCommand[18] = 0x00;
        controlCommand[19] = 0x00;
        controlCommand[20] = 0x00;
        controlCommand[21] = 0x00;
        controlCommand[22] = 0x00;
        uint8_t commandArr[txLength];
        //--[23,24] 帧尾
        // controlCommand[23] = 0xfc;
        // controlCommand[24] = 0x03;
        for(int i=0;i<txLength;i++){
            commandArr[i] = controlCommand[i];
        }
        uart_check.Append_CRC16_Check_Sum(commandArr,txLength);
        controlCommand[23] = commandArr[23];
        controlCommand[24] = commandArr[24];
        this->uartSolver.write(controlCommand);    
    }
    catch (...)
    {
        cout << fontColorWhite << endl;
        // system("play src/roborts_uart/music/uart2.mp3");
        cerr << fontColorRed << "串口无法链接" << endl;
        exit(0);
    }
}
void coal_uart::readData(){
    //--part1 读取串口数据到缓冲区
    size_t receiverSize = uartSolver.available();
    if (receiverSize != 0)
    {
        uint8_t receiver[4095];
        receiverSize = uartSolver.read(receiver, receiverSize);
        for (int index = 0; index < receiverSize; index++)
        {
            uartBuffer.push_back(receiver[index]);
        }
    }

    //--part 2缓冲区溢出处理
    if (uartBuffer.size() > 999)
    {
        uartBuffer.clear();
        // cout<<"清空"<<endl;
    }

    //--part3 缓冲区数据解析
    if (uartBuffer.size() >= 30) // A包24字节+B包16字节
    {
        // 解析数据
        if(uartBuffer.at(0) == 0x05 && uartBuffer.at(1) == 0xfa ){
            unixTimeRecord();
            // CRC校验
            for(int i=0;i<RX_LENGTH;i++)
            {
                uartRxBuffer[i] = uartBuffer.at(i);
            }
            last1 = uartBuffer.at(28);
            last2 = uartBuffer.at(29);

            uart_check.Append_CRC16_Check_Sum(uartRxBuffer,RX_LENGTH);
            if(uartRxBuffer[28] == last1 && uartRxBuffer[29] == last2)
            {
                /* 对下位机数据解码 */
                analyzePackage();
                // Publisher_TimeStamp();
            }
            memset(uartRxBuffer,0,RX_LENGTH);    //数组清0
            uartBuffer.erase(uartBuffer.begin(),uartBuffer.begin()+RX_LENGTH-1);  //将数组清0
        }
        else {
            //帧头帧尾不对代表当前数据无效，所以删除第一个数据，这样的话直接整组数据就无效了
            uartBuffer.pop_front();
        }    
    }

}

void coal_uart::displayTag(char str){
    if (param["uartParam"].at("isDisplay") == 0)
        return;

    timeb t;
    ftime(&t);
    if (str == 'A')
    {
        cout << fontColorBlue << "[" << t.time * 1000 + t.millitm << "]"
            << "sensorDataReception sending to NUC 500hz" << endl;
    }
    else if (str == 'B')
    {
        cout << fontColorYellow << "[" << t.time * 1000 + t.millitm << "]"
            << "judgmentReception sending to NUC 10hz" << endl;
    }
    else if (str == 'C')
    {
        cout << fontColorGreen << "[" << t.time * 1000 + t.millitm << "]"
            << "cmd sending to MCU 500hz" << endl;
    }

    else
    {
    }
}

void coal_uart::analyzePackage(){
    sensorDataReception.chassisSpeed1 = (float)((uartBuffer[2] << 8 | uartBuffer[3]) - 4000) / 1000.0f;
    sensorDataReception.chassisSpeed2 = (float)((uartBuffer[4] << 8 | uartBuffer[5]) - 4000) / 1000.0f;
    sensorDataReception.chassisSpeed3 = (float)((uartBuffer[6] << 8 | uartBuffer[7]) - 4000) / 1000.0f;
    sensorDataReception.chassisSpeed4 = (float)((uartBuffer[8] << 8 | uartBuffer[9]) - 4000) / 1000.0f;
    sensorDataReception.waistAngle = (float)((uartBuffer[10] << 8 | uartBuffer[11]) - 18000) / 100.0f;
    // cout<<sensorDataReception.chassisSpeed1<<" "<<sensorDataReception.chassisSpeed2<<" "<<sensorDataReception.chassisSpeed3<<" "<<sensorDataReception.chassisSpeed4<<" "<<sensorDataReception.waistAngle<<endl;
}

void coal_uart::unixTimeRecord()
{
    static int recordFlag = 0;
    if (!recordFlag)
    {
        timeb tUnix;
        ftime(&tUnix);
        ofstream outfile;
        std::string unixTimeStampPath = ros::package::getPath("coal_uart");
        outfile.open(unixTimeStampPath + "/unixTimeStamp.dat");
        outfile << tUnix.time * 1000 + tUnix.millitm;
        outfile.close();
        recordFlag = 1;
    }
}

coal_uart::~coal_uart()
{

}