#NUC-->STM32 C包:控制指令
#50hz 25字节        /chassisControl
#[0,1] 帧头

#[2,3]全局x轴线速度
float32 chassisSpeed1
#[4,5]
float32 chassisSpeed2
#[6,7]
float32 chassisSpeed3
#[8,9]
float32 chassisSpeed4
#[10,11]腰关节角度
float32 waistAngle 
#[12,13]铲斗控制
uint8 basketControl
# [23,24]帧尾