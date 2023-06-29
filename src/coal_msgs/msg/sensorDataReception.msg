#STM32-->NUC A包:传感器信息 
#500hz   30字节  /sensorDataReception

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


# float32 xAccel 

# float32 yAccel

# float32 mobileChassiAngle 
# # [12,13,14]云台yaw角
# float32 shootPlatformYaw
# #[15,16]云台picth角
# float32 shootPlatformPitch
# #[17]弹速
# float32 bulletVelocity 
# #[18,19,20,21]  时间戳
# uint32 timeStamp

#帧尾[28,29]