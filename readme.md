# FG927
## FG927 ver.ESP32H2
    首先，我要先声明一点，我和铃羽同一天生日，所以这个项目叫做FG927，因为它使用了esp32h2，所以是ver.ESP32H2
这是一个基于墨水瓶的GPS码表项目

功能如下：

- 基于GPS的速度显示（一位小数）
- 基于磁力计并使用GPS修正的骑行方向
- 基于陀螺仪的爬坡角度（不怎么工作，过一个减速带就飞了）
- 单程均速、单程极速、单程里程、单程骑行时间、总里程
- 过去十二个小时内的均速、极速统计（精确到分钟）
- 夜间显示照明（最亮可以当手电）
- GPS修正的utc+8时间
- 单程卡路里计算（假设你70kg）
- 气温、气压、高度
- 停止骑行4分钟后自动待机，监测车辆运动自动唤醒
- 续航约九小时（开灯将更短）、低电量待机（从他不能开机到完全没电大概一周）
- 128Mb flash，BLE组成的骑行路线记录功能（待开发）
- **显示铃羽的图片**

## 使用到的关键硬件

- MCU：ESP32-H2-MINI-1
- 陀螺仪：ICM20948
- 电源管理：AXP2101（DCDC1的默认电压必须是3.3V！！！）
- 气压计：MS5611
- LED管理：LP5009
- 墨水屏：GDEY0154D67
- GPS：ATGM336H-5N31
- 功率计：INA219
- flash：W25Q128JVPIQ（未使用）
- 兼容st7789的lcd屏幕，但没有配套的外壳3d建模文件
- 603035 600mah锂电池
- 陶瓷天线：BWGPSCNX18-18B1

## 使用到的库                                       
- Encoder                                                 
- LP50XX                                             
- ArduinoINA219                                         
- XPowersLib                                               https://github.com/lewisxhe/XPowersLib
- SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/tree/main
- TinyGPSPlus-ESP32                                       
- Adafruit GFX Library                                     
- Adafruit BusIO                                         
- GxEPD2_4G                                                https://github.com/ZinggJM/GxEPD2_4G

## 3D打印外壳
外壳的盖是双色打印的，不双色问题也不大反正，层高是0.08mm，没试过其他的能不能成功，总之精细一点没坏处，他也不需要承受什么巨大的应变。

支架有两种，一种是伸出去的，需要组装，一种是放在把上的，可以根据喜好打印，最好用abs之类的玻璃化转变温度高一点的，pla可能晒一晒就弯了。

需要额外准备M5的螺丝和螺母，以及M3高3mm外径4mm的滚花螺母和M3高5mm头直径6mm厚0.8mm的平头螺丝，M3*8的内六角螺丝。

## pcb

https://oshwhub.com/donchaka0/gps_suzuha

注意，电阻R38用于控制AXP2101的充电模式为LDO模式，以空出DCDC5为GPS供电；电阻R41是编码器PUSH的上拉电阻，已通过ESP32的内部上拉代替，焊接会导致待机时引脚随ALDO3关闭而拉低，这是个BUG。**上述两个电阻不焊接！！！！！**

## 调试

需要使用3.0以上版本的ESP32开发板库才能编译ESP32H2的文件，请一定注意。

测试焊接是否成功时，可以去掉注释BIANCHI.h中#define D_BUG的注释，即可从串口中返回报错数据（除了通讯失败以外的报错信息都很混乱）。